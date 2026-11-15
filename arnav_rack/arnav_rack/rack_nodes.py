"""
ROS 2 nodes of the ARNAV rack stack:

  scan_merger       /scan_front --> /scan around the robot centre, masks the carried rack's legs
  rack_detector     /scan --> /rack/detections (+ RViz markers)
  jack_controller   ramps the lift plate height command
  rack_docking      PickRack / PlaceRack action servers
  rfid_reader_sim   simulated floor-tag reader

All algorithms live in rack_core.py (tested offline).
"""
import math
import os
import threading
import time

from arnav_interfaces.action import PickRack, PlaceRack
from arnav_interfaces.msg import RackDetection, RackDetectionArray, RfidTag
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from arnav_rack.rack_core import (ApproachSession, AxisParams, DetectorParams, MapRefiner, RackTracker,
                                  RetreatSession, SessionParams, beams_to_xy, detect_racks, merge)


# ====================================================================================================
# scan_merger.py
# ====================================================================================================

# ROS node: take the single front LiDAR (/scan_front) and republish it as /scan around the robot centre.

class ScanMerger(Node):
    def __init__(self):
        super().__init__('scan_merger')
        self.declare_parameter('mount_front', [0.37, 0.0, 0.0])
        self.declare_parameter('output_frame', 'scan_link')
        self.declare_parameter('num_bins', 720)
        self.declare_parameter('range_min', 0.10)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('self_box', [0.40, 0.30])
        self.declare_parameter('rack_box', [0.55, 0.55])
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('max_age', 0.5)

        self.mounts = {'front': tuple(self.get_parameter('mount_front').value)}
        self.frame = self.get_parameter('output_frame').value
        self.bins = int(self.get_parameter('num_bins').value)
        self.rmin = float(self.get_parameter('range_min').value)
        self.rmax = float(self.get_parameter('range_max').value)
        self.self_box = tuple(self.get_parameter('self_box').value)
        self.rack_box = tuple(self.get_parameter('rack_box').value)
        self.max_age_ns = int(float(self.get_parameter('max_age').value) * 1e9)

        self.latest = {'front': None}                 # key -> (stamp_ns, points)
        self.last_published = None
        self.carrying = False

        self.create_subscription(LaserScan, 'scan_front', lambda m: self._on_scan('front', m), qos_profile_sensor_data)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Bool, 'arnav/carrying', self._on_carrying, latched)
        self.pub = self.create_publisher(LaserScan, 'scan', QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._tick)
        self.get_logger().info('scan_merger: /scan_front -> /scan')

    def _on_carrying(self, msg):
        self.carrying = bool(msg.data)

    def _on_scan(self, key, msg):
        pts = beams_to_xy(msg.ranges, msg.angle_min, msg.angle_increment,
                                     max(msg.range_min, self.rmin), min(msg.range_max, self.rmax),
                                     self.mounts[key])
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self.latest[key] = (stamp, pts)

    def _tick(self):
        now = self.get_clock().now().nanoseconds
        sets, stamps = [], []
        for key in ('front',):
            item = self.latest[key]
            if item is None or now - item[0] > self.max_age_ns:
                continue
            stamps.append(item[0])
            sets.append(item[1])
        if not sets:
            return
        marker = self.latest['front'][0]
        if marker == self.last_published:
            return                                  # nothing new since the last merged scan
        self.last_published = marker
        ranges, a0, da = merge(sets, self.bins, self.rmax, self.self_box, self.rack_box,
                                          self.carrying)
        out = LaserScan()
        newest = max(stamps)
        out.header.stamp.sec = int(newest // 10**9)
        out.header.stamp.nanosec = int(newest % 10**9)
        out.header.frame_id = self.frame
        out.angle_min = float(a0)
        out.angle_max = float(a0 + (self.bins - 1) * da)
        out.angle_increment = float(da)
        out.time_increment = 0.0
        out.scan_time = 1.0 / float(self.get_parameter('rate').value)
        out.range_min = 0.05
        out.range_max = self.rmax
        out.ranges = np.asarray(ranges, dtype=np.float32).tolist()
        self.pub.publish(out)


# ====================================================================================================
# rack_detector.py
# ====================================================================================================

# ROS node: find racks in the merged LiDAR scan by the pattern of their four legs (no markers, no camera).

class RackDetector(Node):
    def __init__(self):
        super().__init__('rack_detector')
        d = DetectorParams()
        self.declare_parameter('output_frame', 'base_footprint')
        self.declare_parameter('rack_side', d.side)
        self.declare_parameter('leg_size', d.leg_size)
        self.declare_parameter('tol_side', d.tol_side)
        self.declare_parameter('tol_match', d.tol_match)
        self.declare_parameter('max_range', d.max_range)
        self.declare_parameter('leg_max_extent', d.leg_max_extent)
        self.declare_parameter('free_margin', d.free_margin)
        self.declare_parameter('publish_markers', True)

        self.frame = self.get_parameter('output_frame').value
        self.params = DetectorParams(
            side=float(self.get_parameter('rack_side').value),
            leg_size=float(self.get_parameter('leg_size').value),
            tol_side=float(self.get_parameter('tol_side').value),
            tol_match=float(self.get_parameter('tol_match').value),
            max_range=float(self.get_parameter('max_range').value),
            leg_max_extent=float(self.get_parameter('leg_max_extent').value),
            free_margin=float(self.get_parameter('free_margin').value),
        )
        self.markers = bool(self.get_parameter('publish_markers').value)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(RackDetectionArray, 'rack/detections', 10)
        self.mpub = self.create_publisher(MarkerArray, 'rack/markers', QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE))
        self.get_logger().info(f'rack_detector: looking for racks with legs {self.params.side:.2f} m apart')

    def _on_scan(self, msg):
        racks, legs = detect_racks(msg.ranges, msg.angle_min, msg.angle_increment,
                                   msg.range_min, msg.range_max, self.params)
        out = RackDetectionArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame
        for r in racks:
            det = RackDetection()
            det.header = out.header
            det.pose.x, det.pose.y, det.pose.theta = float(r.x), float(r.y), float(r.yaw)
            det.legs_matched = int(r.legs)
            det.confidence = float(r.confidence)
            det.side = float(self.params.side)
            out.racks.append(det)
        self.pub.publish(out)
        if self.markers:
            self._publish_markers(out.header, racks, legs)

    def _publish_markers(self, header, racks, legs):
        arr = MarkerArray()
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        mid = 0
        for r in racks:
            m = Marker()
            m.header = header
            m.ns, m.id = 'rack', mid
            mid += 1
            m.type = Marker.CUBE
            side = self.params.side + self.params.leg_size
            m.pose.position.x, m.pose.position.y, m.pose.position.z = r.x, r.y, 0.05
            m.pose.orientation.z = math.sin(r.yaw / 2.0)
            m.pose.orientation.w = math.cos(r.yaw / 2.0)
            m.scale.x = m.scale.y = side
            m.scale.z = 0.10
            m.color.r, m.color.g, m.color.b = (0.1, 0.9, 0.2) if r.legs == 4 else (1.0, 0.8, 0.1)
            m.color.a = 0.45
            m.lifetime.nanosec = 500_000_000
            arr.markers.append(m)
            t = Marker()
            t.header = header
            t.ns, t.id = 'rack_label', mid
            mid += 1
            t.type = Marker.TEXT_VIEW_FACING
            t.pose.position.x, t.pose.position.y, t.pose.position.z = r.x, r.y, 0.4
            t.scale.z = 0.18
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = f'rack ({r.legs} legs)'
            t.lifetime.nanosec = 500_000_000
            arr.markers.append(t)
        if len(legs):
            s = Marker()
            s.header = header
            s.ns, s.id = 'legs', mid
            s.type = Marker.SPHERE_LIST
            s.scale.x = s.scale.y = s.scale.z = 0.07
            s.color.r, s.color.g, s.color.b, s.color.a = 0.2, 0.5, 1.0, 0.9
            from geometry_msgs.msg import Point
            s.points = [Point(x=float(p[0]), y=float(p[1]), z=0.05) for p in legs]
            s.lifetime.nanosec = 500_000_000
            arr.markers.append(s)
        self.mpub.publish(arr)


# ====================================================================================================
# jack_controller.py
# ====================================================================================================

# ROS node: smooth height command for the lift plate.

class JackController(Node):
    def __init__(self):
        super().__init__('jack_controller')
        self.declare_parameter('joint_name', 'jack_joint')
        self.declare_parameter('speed', 0.02)
        self.declare_parameter('min_height', 0.0)
        self.declare_parameter('max_height', 0.08)
        self.declare_parameter('lift_height', 0.07)
        self.declare_parameter('rate', 30.0)
        self.joint = self.get_parameter('joint_name').value
        self.speed = float(self.get_parameter('speed').value)
        self.lo = float(self.get_parameter('min_height').value)
        self.hi = float(self.get_parameter('max_height').value)
        self.lift_height = float(self.get_parameter('lift_height').value)
        self.dt = 1.0 / float(self.get_parameter('rate').value)

        self.target = 0.0
        self.cmd = 0.0
        self.position = None

        self.create_subscription(Float64, 'arnav/jack/target', self._on_target, 10)
        self.create_subscription(JointState, 'joint_states', self._on_joints, 10)
        self.cmd_pub = self.create_publisher(Float64, 'arnav/jack/cmd_pos', 10)
        self.pos_pub = self.create_publisher(Float64, 'arnav/jack/position', 10)
        self.state_pub = self.create_publisher(String, 'arnav/jack/state', 10)
        self.create_service(Trigger, 'arnav/jack/lift', self._lift)
        self.create_service(Trigger, 'arnav/jack/lower', self._lower)
        self.create_timer(self.dt, self._tick)

    def _clamp(self, v):
        return max(self.lo, min(self.hi, v))

    def _on_target(self, msg):
        self.target = self._clamp(float(msg.data))

    def _on_joints(self, msg):
        if self.joint in msg.name:
            self.position = float(msg.position[msg.name.index(self.joint)])
            self.pos_pub.publish(Float64(data=self.position))

    def _lift(self, _req, res):
        self.target = self._clamp(self.lift_height)
        res.success, res.message = True, f'raising to {self.target:.3f} m'
        return res

    def _lower(self, _req, res):
        self.target = self.lo
        res.success, res.message = True, 'lowering'
        return res

    def _tick(self):
        step = self.speed * self.dt
        if abs(self.target - self.cmd) <= step:
            self.cmd = self.target
        else:
            self.cmd += step if self.target > self.cmd else -step
        self.cmd_pub.publish(Float64(data=self.cmd))
        if self.position is None:
            state = 'unknown'
        elif abs(self.cmd - self.target) > 1e-6:
            state = 'moving'
        elif self.target > 0.5 * (self.lo + self.hi) * 0.0 + 0.02 and abs(self.position - self.target) < 0.006:
            state = 'raised'
        elif self.target <= self.lo + 1e-6 and self.position < self.lo + 0.006:
            state = 'lowered'
        else:
            state = 'moving'
        self.state_pub.publish(String(data=state))


# ====================================================================================================
# rack_docking.py
# ====================================================================================================

# ROS node: PickRack and PlaceRack action servers.

def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    """pose b (in frame A) expressed in A's parent: a = (x, y, yaw) of A, b = (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


class RackDocking(Node):
    def __init__(self):
        super().__init__('rack_docking')
        g = ReentrantCallbackGroup()
        for name, val in [('odom_frame', 'odom'), ('map_frame', 'map'), ('base_frame', 'base_footprint'),
                          ('control_rate', 20.0), ('default_gate', 0.6), ('lift_height', 0.07),
                          ('jack_tol', 0.012), ('jack_timeout', 30.0), ('settle_time', 1.0),
                          ('retreat_distance', 1.3), ('check_slot_free', True), ('slot_free_radius', 0.5),
                          ('v_max', 0.25), ('v_min', 0.04), ('w_max', 0.6), ('reverse_v', 0.20),
                          ('k_psi', 2.0), ('k_e', 2.0), ('rfid_wait', 3.0), ('rfid_max_age', 1.0), ('layout_file', ''),
                          ('refine_slot_pose', True), ('refine_max_shift', 0.45), ('refine_scans', 5)]:
            self.declare_parameter(name, val)
        P = lambda n: self.get_parameter(n).value  # noqa: E731
        self.odom, self.map, self.base = P('odom_frame'), P('map_frame'), P('base_frame')
        self.dt = 1.0 / float(P('control_rate'))
        self.axis = AxisParams(v_max=float(P('v_max')), v_min=float(P('v_min')), w_max=float(P('w_max')),
                               reverse_v=float(P('reverse_v')), k_psi=float(P('k_psi')), k_e=float(P('k_e')))

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.jack_pub = self.create_publisher(Float64, 'arnav/jack/target', 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        self.carry_pub = self.create_publisher(Bool, 'arnav/carrying', latched)
        self.carry_pub.publish(Bool(data=False))
        self.create_subscription(RackDetectionArray, 'rack/detections', self._on_det, 10, callback_group=g)
        self.create_subscription(Float64, 'arnav/jack/position', self._on_jack, 10, callback_group=g)
        self.create_subscription(RfidTag, 'rfid/tag', self._on_tag, 10, callback_group=g)
        # scan-to-map pose refinement before a rack is put down (AMCL alone is off by several cm up to ~0.3 m)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        self.refiner = None
        self.scan_msg = None
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos, callback_group=g)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data, callback_group=g)
        self.epc = self._load_epc_aliases()
        self.last_tag, self.last_tag_time = None, 0.0

        self.lock = threading.Lock()
        self.det_seq = 0
        self.det_msg = None
        self.jack_pos = None
        self.busy = False

        self.pick_srv = ActionServer(self, PickRack, 'arnav/pick_rack', self._exec_pick,
                                     goal_callback=self._goal, cancel_callback=self._cancel, callback_group=g)
        self.place_srv = ActionServer(self, PlaceRack, 'arnav/place_rack', self._exec_place,
                                      goal_callback=self._goal, cancel_callback=self._cancel, callback_group=g)
        self.get_logger().info('rack_docking ready: /arnav/pick_rack, /arnav/place_rack')

    # ---- plumbing -------------------------------------------------------------------------------
    def _goal(self, _req):
        if self.busy:
            self.get_logger().warn('rejecting goal: docking already running')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel(self, _gh):
        return CancelResponse.ACCEPT

    def _on_det(self, msg):
        with self.lock:
            self.det_msg = msg
            self.det_seq += 1

    def _on_jack(self, msg):
        self.jack_pos = float(msg.data)

    def _on_map(self, msg):
        try:
            self.refiner = MapRefiner(msg.data, msg.info.width, msg.info.height, msg.info.resolution,
                                      msg.info.origin.position.x, msg.info.origin.position.y)
            self.get_logger().info('map received: slot pose refinement available')
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'could not build the refinement map: {e}')

    def _on_scan(self, msg):
        self.scan_msg = msg

    def _refined_slot(self, slot_map, slot_odom):
        """Slot pose in odom, corrected by matching the scan to the map.  Falls back to `slot_odom` if it can't.

        slot_map = slot (x, y, yaw) in the map frame, slot_odom = the same from the AMCL transform.
        """
        if not bool(self.get_parameter('refine_slot_pose').value) or self.refiner is None:
            return slot_odom
        robot_map = self._pose(self.map)
        robot_odom = self._pose(self.odom)
        if robot_map is None or robot_odom is None:
            return slot_odom
        deltas = []
        last = -1
        t0 = time.monotonic()
        want = int(self.get_parameter('refine_scans').value)
        while len(deltas) < want and time.monotonic() - t0 < 4.0 and rclpy.ok():
            m = self.scan_msg
            if m is None or id(m) == last:
                time.sleep(0.05)
                continue
            last = id(m)
            pose, info = self.refiner.refine(robot_map, m.ranges, m.angle_min, m.angle_increment)
            if info['ok']:
                deltas.append((pose[0] - robot_map[0], pose[1] - robot_map[1], pose[2] - robot_map[2]))
        if len(deltas) < max(2, want // 2):
            self.get_logger().warn('slot refinement: scan does not match the map here, using the AMCL pose')
            return slot_odom
        d = tuple(sorted(v)[len(v) // 2] for v in zip(*deltas))      # median of the scans
        if math.hypot(d[0], d[1]) > float(self.get_parameter('refine_max_shift').value):
            self.get_logger().warn(f'slot refinement shift {math.hypot(d[0], d[1]):.2f} m too large, ignored')
            return slot_odom
        better = (robot_map[0] + d[0], robot_map[1] + d[1], robot_map[2] + d[2])
        # where the slot is, seen from the better robot pose, then put back in odom with the robot's odom pose
        c, s = math.cos(better[2]), math.sin(better[2])
        dx, dy = slot_map[0] - better[0], slot_map[1] - better[1]
        rel = (c * dx + s * dy, -s * dx + c * dy, slot_map[2] - better[2])
        out = compose(robot_odom, rel)
        self.get_logger().info(
            f'slot refined: AMCL was off by ({-d[0] * 100:+.1f}, {-d[1] * 100:+.1f}) cm, '
            f'slot moves {math.hypot(out[0] - slot_odom[0], out[1] - slot_odom[1]) * 100:.1f} cm in odom')
        return out

    # ---- RFID -----------------------------------------------------------------------------------
    def _load_epc_aliases(self):
        """reader id (EPC) -> short tag id, from the rfid.epc block of layout.yaml.  Empty if unavailable."""
        path = self.get_parameter('layout_file').value
        try:
            if not path:
                from ament_index_python.packages import get_package_share_directory
                path = f"{get_package_share_directory('arnav_sim')}/config/layout.yaml"
            with open(path) as f:
                epc = (yaml.safe_load(f).get('rfid') or {}).get('epc') or {}
            return {str(k).upper(): str(v).lower() for k, v in epc.items()}
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'no EPC aliases loaded ({e}); tag ids are used as reported')
            return {}

    def _norm_tag(self, raw):
        t = str(raw).strip()
        return self.epc.get(t.upper(), t.lower())

    def _on_tag(self, msg):
        self.last_tag, self.last_tag_time = self._norm_tag(msg.tag_id), time.monotonic()

    def _check_tag(self, expected, gh=None):
        """Wait up to rfid_wait s for `expected` under the robot.  Returns (ok, message).  Empty `expected` = skip."""
        if not expected:
            return True, ''
        want = self._norm_tag(expected)
        wait = float(self.get_parameter('rfid_wait').value)
        max_age = float(self.get_parameter('rfid_max_age').value)
        t0 = time.monotonic()
        seen = None
        while time.monotonic() - t0 < wait and rclpy.ok():
            if self.last_tag is not None and time.monotonic() - self.last_tag_time <= max_age:
                seen = self.last_tag
                if seen == want:
                    return True, f'RFID tag {want} confirmed'
            time.sleep(0.1)
        if seen is None:
            return False, f'RFID: expected tag {want} but no tag was read (is /rfid/tag publishing? reader range?)'
        return False, f'RFID: expected tag {want} but the robot is over tag {seen}'

    def _pose(self, frame, stamp=None):
        """Robot (x, y, yaw) in `frame`, at `stamp` if possible, else the latest."""
        tries = []
        if stamp is not None:
            tries.append(rclpy.time.Time.from_msg(stamp))
        tries.append(rclpy.time.Time())
        for t in tries:
            try:
                tr = self.tf.lookup_transform(frame, self.base, t, timeout=Duration(seconds=0.0))
                return (tr.transform.translation.x, tr.transform.translation.y, yaw_of(tr.transform.rotation))
            except Exception:  # noqa: BLE001  (extrapolation / lookup errors: try the next time)
                continue
        return None

    def _frame_pose(self, target, source):
        """Pose of frame `source` in frame `target`."""
        try:
            tr = self.tf.lookup_transform(target, source, rclpy.time.Time(), timeout=Duration(seconds=0.0))
            return (tr.transform.translation.x, tr.transform.translation.y, yaw_of(tr.transform.rotation))
        except Exception:  # noqa: BLE001
            return None

    def _drive(self, v, w):
        m = Twist()
        m.linear.x, m.angular.z = float(v), float(w)
        self.cmd_pub.publish(m)

    def _stop(self):
        self._drive(0.0, 0.0)

    def _detections_in_odom(self, last_seq):
        """New detections as (x, y, yaw, legs) in the odom frame."""
        with self.lock:
            msg, seq = self.det_msg, self.det_seq
        if msg is None or seq == last_seq:
            return seq, None
        robot = self._pose(self.odom, msg.header.stamp)
        if robot is None:
            return seq, None
        out = []
        for r in msg.racks:
            x, y, yaw = compose(robot, (r.pose.x, r.pose.y, r.pose.theta))
            out.append((x, y, yaw, int(r.legs_matched)))
        return seq, out

    def _hint_odom(self, pose_stamped):
        p = pose_stamped.pose
        frame = pose_stamped.header.frame_id or self.map
        hint = (p.position.x, p.position.y, yaw_of(p.orientation))
        if frame == self.odom:
            return hint
        t = self._frame_pose(self.odom, frame)
        return None if t is None else compose(t, hint)

    def _set_jack(self, height, timeout):
        self.jack_pub.publish(Float64(data=float(height)))
        tol = float(self.get_parameter('jack_tol').value)
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and rclpy.ok():
            if self.jack_pos is not None and abs(self.jack_pos - height) <= tol:
                return True
            time.sleep(0.05)
        return False

    def _jack_info(self):
        return 'no /arnav/jack/position messages received' if self.jack_pos is None else f'plate height is {self.jack_pos:.3f} m'

    def _carrying(self, flag):
        self.carry_pub.publish(Bool(data=bool(flag)))

    # ---- pick -----------------------------------------------------------------------------------
    def _exec_pick(self, gh):
        self.busy = True
        res = PickRack.Result()
        fb = PickRack.Feedback()
        try:
            hint = self._hint_odom(gh.request.rack_pose)
            if hint is None:
                return self._abort(gh, res, 'no TF between map and odom (is localisation running?)')
            gate = gh.request.gate_radius or float(self.get_parameter('default_gate').value)
            sess = ApproachSession(tracker=RackTracker(hint[0], hint[1], hint[2], gate=gate),
                                   axis_params=self.axis, params=SessionParams(), t0=time.monotonic())
            seq = -1
            while rclpy.ok():
                if gh.is_cancel_requested:
                    self._stop()
                    gh.canceled()
                    res.message = 'cancelled'
                    return res
                robot = self._pose(self.odom)
                if robot is None:
                    time.sleep(self.dt)
                    continue
                seq, dets = self._detections_in_odom(seq)
                if dets is not None:
                    sess.update_detections(dets)
                cmd = sess.step(robot, time.monotonic())
                fb.state, fb.distance_to_go = cmd.state, float(cmd.ex)
                fb.lateral_error, fb.heading_error = float(cmd.e), float(cmd.psi)
                gh.publish_feedback(fb)
                if cmd.error:
                    self._stop()
                    return self._abort(gh, res, cmd.error)
                if cmd.done:
                    break
                self._drive(cmd.v, cmd.w)
                time.sleep(self.dt)
            self._stop()
            res.final_lateral_error, res.final_heading_error = float(cmd.e), float(cmd.psi)
            fb.state = 'CHECKING_RFID'
            gh.publish_feedback(fb)
            tag_ok, tag_msg = self._check_tag(gh.request.expected_tag)
            if not tag_ok:
                return self._abort(gh, res, tag_msg)
            fb.state = 'LIFTING'
            gh.publish_feedback(fb)
            ok = self._set_jack(float(self.get_parameter('lift_height').value),
                                float(self.get_parameter('jack_timeout').value))
            if not ok:
                self.jack_pub.publish(Float64(data=0.0))
                return self._abort(gh, res, f'lift plate did not reach height: {self._jack_info()}')
            time.sleep(float(self.get_parameter('settle_time').value))
            self._carrying(True)
            gh.succeed()
            res.success, res.message = True, 'rack lifted'
            return res
        finally:
            self.busy = False

    # ---- place ----------------------------------------------------------------------------------
    def _slot_occupied(self, slot):
        """True if the LiDAR sees a rack at the slot (our own rack is masked out of the scan while carried)."""
        with self.lock:
            msg = self.det_msg
        if msg is None:
            return False
        robot = self._pose(self.odom, msg.header.stamp)
        if robot is None:
            return False
        rad = float(self.get_parameter('slot_free_radius').value)
        for r in msg.racks:
            x, y, _ = compose(robot, (r.pose.x, r.pose.y, 0.0))
            if math.hypot(x - slot[0], y - slot[1]) < rad:
                return True
        return False

    def _exec_place(self, gh):
        self.busy = True
        res = PlaceRack.Result()
        fb = PlaceRack.Feedback()
        try:
            slot = self._hint_odom(gh.request.slot_pose)
            if slot is None:
                return self._abort(gh, res, 'no TF between map and odom (is localisation running?)')
            fb.state = 'CHECKING_SLOT'
            gh.publish_feedback(fb)
            time.sleep(0.6)    # let a fresh detection arrive
            slot_map = (gh.request.slot_pose.pose.position.x, gh.request.slot_pose.pose.position.y,
                        yaw_of(gh.request.slot_pose.pose.orientation))
            if (gh.request.slot_pose.header.frame_id or self.map) == self.map:
                slot = self._refined_slot(slot_map, slot)
            if bool(self.get_parameter('check_slot_free').value) and self._slot_occupied(slot):
                return self._abort(gh, res, 'slot is occupied by another rack')
            sess = ApproachSession(static_target=slot, axis_params=self.axis, params=SessionParams(),
                                   t0=time.monotonic())
            while rclpy.ok():
                if gh.is_cancel_requested:
                    self._stop()
                    gh.canceled()
                    res.message = 'cancelled'
                    return res
                robot = self._pose(self.odom)
                if robot is None:
                    time.sleep(self.dt)
                    continue
                cmd = sess.step(robot, time.monotonic())
                fb.state, fb.distance_to_go = cmd.state, float(cmd.ex)
                gh.publish_feedback(fb)
                if cmd.error:
                    self._stop()
                    return self._abort(gh, res, cmd.error)
                if cmd.done:
                    break
                self._drive(cmd.v, cmd.w)
                time.sleep(self.dt)
            self._stop()
            res.position_error = float(math.hypot(cmd.ex, cmd.e))
            res.heading_error = float(cmd.psi)
            fb.state = 'CHECKING_RFID'
            gh.publish_feedback(fb)
            tag_ok, tag_msg = self._check_tag(gh.request.expected_tag)
            if not tag_ok:
                return self._abort(gh, res, tag_msg + ' (rack stays on the plate)')
            fb.state = 'LOWERING'
            gh.publish_feedback(fb)
            ok = self._set_jack(0.0, float(self.get_parameter('jack_timeout').value))
            if not ok:
                return self._abort(gh, res, f'lift plate did not come down: {self._jack_info()}')
            time.sleep(float(self.get_parameter('settle_time').value))
            self._carrying(False)
            if gh.request.retreat:
                ret = RetreatSession(slot, float(self.get_parameter('retreat_distance').value),
                                     axis_params=self.axis, t0=time.monotonic())
                while rclpy.ok():
                    robot = self._pose(self.odom)
                    if robot is None:
                        time.sleep(self.dt)
                        continue
                    cmd = ret.step(robot, time.monotonic())
                    fb.state, fb.distance_to_go = cmd.state, float(max(0.0, ret.distance - cmd.ex))
                    gh.publish_feedback(fb)
                    if cmd.error or cmd.done:
                        break
                    self._drive(cmd.v, cmd.w)
                    time.sleep(self.dt)
                self._stop()
                if cmd.error:
                    return self._abort(gh, res, cmd.error)
            gh.succeed()
            res.success, res.message = True, 'rack placed'
            return res
        finally:
            self.busy = False

    def _abort(self, gh, res, msg):
        self.get_logger().error(msg)
        self._stop()
        gh.abort()
        res.success, res.message = False, msg
        return res


# ====================================================================================================
# rfid_reader_sim.py
# ====================================================================================================

# Simulated RFID reader (the sim has no RFID).  Publishes /rfid/tag while the robot centre is over a floor tag.

def tag_positions(layout):
    slots = {k: (float(v['x']), float(v['y'])) for k, v in layout['slots'].items()}
    return {str(t): slots[s] for t, s in (layout.get('rfid', {}).get('tags') or {}).items() if s in slots}


class RfidReaderSim(Node):
    def __init__(self):
        super().__init__('rfid_reader_sim')
        from ament_index_python.packages import get_package_share_directory
        default = os.path.join(get_package_share_directory('arnav_sim'), 'config', 'layout.yaml')
        self.declare_parameter('layout_file', default)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('read_range', -1.0)          # <0: use rfid.read_range from the layout
        with open(self.get_parameter('layout_file').value) as f:
            layout = yaml.safe_load(f)
        self.tags = tag_positions(layout)
        rr = float(self.get_parameter('read_range').value)
        self.range = rr if rr > 0 else float(layout.get('rfid', {}).get('read_range', 0.15))
        self.map, self.base = self.get_parameter('map_frame').value, self.get_parameter('base_frame').value
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(RfidTag, 'rfid/tag', 10)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._tick)
        self.get_logger().info(f'rfid_reader_sim: {len(self.tags)} tags, read range {self.range:.2f} m')

    def _tick(self):
        try:
            t = self.tf.lookup_transform(self.map, self.base, rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return
        x, y = t.transform.translation.x, t.transform.translation.y
        for tag, (tx, ty) in self.tags.items():
            d = math.hypot(x - tx, y - ty)
            if d <= self.range:
                m = RfidTag()
                m.header.stamp = self.get_clock().now().to_msg()
                m.header.frame_id = self.base
                m.tag_id = tag
                m.rssi = float(1.0 - d / self.range)
                self.pub.publish(m)


# ====================================================================================================
# entry points
# ====================================================================================================

def _spin(node_cls):
    rclpy.init()
    node = node_cls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


def scan_merger_main():
    _spin(ScanMerger)


def rack_detector_main():
    _spin(RackDetector)


def jack_controller_main():
    _spin(JackController)


def rack_docking_main():
    rclpy.init()
    node = RackDocking()
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


def rfid_reader_sim_main():
    _spin(RfidReaderSim)
