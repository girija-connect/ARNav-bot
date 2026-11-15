"""
ROS 2 mission commands of the ARNAV warehouse:

  ros2 run arnav_mission move_rack <from> <to> [--home] [--force]
  ros2 run arnav_mission run_tasks [tasks.yaml] [--home]
  ros2 run arnav_mission goto home | slot_05 | w1 | X Y [YAW_DEG]
  ros2 run arnav_mission rfid_cmd --pick 01 place w1
  ros2 run arnav_mission show_layout

Layout and RFID logic live in mission_core.py (tested offline).
"""
import argparse
import math
import os
import sys
import time

from action_msgs.msg import GoalStatus
from arnav_interfaces.action import PickRack, PlaceRack
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
import rclpy.utilities
from tf2_ros import Buffer, TransformListener
import yaml

from arnav_mission.mission_core import (GridMap, Layout, parse_command, share_dir, TagError,
                                        TagRegistry)


# ====================================================================================================
# Mission
# ====================================================================================================

SMALL = '[[0.37, 0.28], [0.37, -0.28], [-0.37, -0.28], [-0.37, 0.28]]'          # robot alone
LARGE = '[[0.47, 0.47], [0.47, -0.47], [-0.47, -0.47], [-0.47, 0.47]]'          # robot + carried rack


class MissionError(RuntimeError):
    pass


def make_pose(x, y, yaw, stamp):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = stamp
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    return p


class Mission:
    def __init__(self, log=print):
        rclpy.init()
        self.nav = BasicNavigator()
        # everything else runs on Gazebo time; this node must too or TF and goal stamps disagree
        self.nav.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.log = log
        self.layout = Layout(grid=GridMap())
        self.tags = TagRegistry(self.layout.d)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self.nav)
        self.pick_client = ActionClient(self.nav, PickRack, 'arnav/pick_rack')
        self.place_client = ActionClient(self.nav, PlaceRack, 'arnav/place_rack')
        self.fp_clients = {
            'local': self.nav.create_client(SetParameters, '/local_costmap/local_costmap/set_parameters'),
            'global': self.nav.create_client(SetParameters, '/global_costmap/global_costmap/set_parameters'),
        }

    # ---- setup ----------------------------------------------------------------------------------
    def wait_ready(self):
        self.log('waiting for Nav2 ...')
        self.nav.initial_pose_received = True      # AMCL already has its initial pose from nav2_params.yaml
        self.nav.waitUntilNav2Active()
        for name, cl in (('pick_rack', self.pick_client), ('place_rack', self.place_client)):
            self.log(f'waiting for /arnav/{name} ...')
            if not cl.wait_for_server(timeout_sec=60.0):
                raise MissionError(f'/arnav/{name} not available: is the rack stack running?')
        self.log('ready')

    def robot_xy(self, wait=15.0):
        t0, err = time.monotonic(), None
        while time.monotonic() - t0 < wait:
            try:
                t = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                return (t.transform.translation.x, t.transform.translation.y)
            except Exception as e:  # noqa: BLE001
                err = e
                rclpy.spin_once(self.nav, timeout_sec=0.1)
        raise MissionError(f'no map -> base_footprint transform after {wait:.0f} s ({err}). '
                           'Check: ros2 run tf2_ros tf2_echo map base_footprint')

    # ---- Nav2 -----------------------------------------------------------------------------------
    def set_footprint(self, carrying):
        value = LARGE if carrying else SMALL
        for name, cl in self.fp_clients.items():
            if not cl.wait_for_service(timeout_sec=2.0):
                self.log(f'  (could not reach the {name} costmap to change the footprint)')
                continue
            req = SetParameters.Request()
            req.parameters = [ParamMsg(name='footprint', value=ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value=value))]
            fut = cl.call_async(req)
            rclpy.spin_until_future_complete(self.nav, fut, timeout_sec=3.0)

    def go(self, x, y, yaw, timeout=240.0, what='goal'):
        self.log(f'  driving to {what} ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)')
        self.nav.goToPose(make_pose(x, y, yaw, self.nav.get_clock().now().to_msg()))
        t0, last = time.monotonic(), 0.0
        while not self.nav.isTaskComplete():
            fb = self.nav.getFeedback()
            if fb and time.monotonic() - last > 6.0:
                last = time.monotonic()
                self.log(f'    {fb.distance_remaining:.1f} m to go')
            if time.monotonic() - t0 > timeout:
                self.nav.cancelTask()
                raise MissionError(f'timed out going to {what}')
        res = self.nav.getResult()
        if res != TaskResult.SUCCEEDED:
            raise MissionError(f'Nav2 could not reach {what} ({res})')

    def go_staging(self, slot, what):
        """Try the staging poses of a slot, best first, until one is reachable.  Returns the entry yaw."""
        options = self.layout.entries(slot, self.robot_xy())
        if not options:
            raise MissionError(f'no free staging pose for {slot}')
        last = None
        for yaw, st in options[:3]:
            try:
                self.go(st[0], st[1], st[2], what=f'{what} staging')
                return yaw
            except MissionError as e:
                last = e
                self.log(f'  {e}; trying another side')
        raise last

    # ---- rack actions ---------------------------------------------------------------------------
    def _run(self, client, goal, what):
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.nav, fut)
        gh = fut.result()
        if not gh.accepted:
            raise MissionError(f'{what} goal rejected (another docking running?)')
        rfut = gh.get_result_async()
        rclpy.spin_until_future_complete(self.nav, rfut)
        r = rfut.result()
        if r.status != GoalStatus.STATUS_SUCCEEDED or not r.result.success:
            raise MissionError(f'{what} failed: {r.result.message}')
        return r.result

    def pick(self, slot, yaw, tag=''):
        cx, cy = self.layout.resolve(slot)
        goal = PickRack.Goal()
        goal.expected_tag = tag or ''
        goal.rack_pose = make_pose(cx, cy, yaw, self.nav.get_clock().now().to_msg())
        self.log(f'  picking the rack at {slot}')
        r = self._run(self.pick_client, goal, 'pick')
        self.log(f'  lifted (lateral {r.final_lateral_error * 100:+.1f} cm, heading {math.degrees(r.final_heading_error):+.1f} deg)')

    def place(self, slot, yaw, retreat=True, tag=''):
        cx, cy = self.layout.resolve(slot)
        goal = PlaceRack.Goal()
        goal.expected_tag = tag or ''
        goal.slot_pose = make_pose(cx, cy, yaw, self.nav.get_clock().now().to_msg())
        goal.retreat = retreat
        self.log(f'  placing the rack in {slot}')
        r = self._run(self.place_client, goal, 'place')
        self.log(f'  placed ({r.position_error * 100:.1f} cm from the slot centre)')

    # ---- the whole job --------------------------------------------------------------------------
    def move_rack(self, src, dst, go_home=False, force=False, verify_rfid=False):
        lay = self.layout
        s, d = lay.slot_name(src), lay.slot_name(dst)
        if s not in lay.occupied():
            raise MissionError(f'{s} has no rack (racks: {sorted(lay.occupied())})')
        if d in lay.occupied() and not force:
            raise MissionError(f'{d} is already occupied by {lay.occupied()[d]}')
        s_tag = (self.tags.tag_of(s) or '') if verify_rfid else ''
        d_tag = (self.tags.tag_of(d) or '') if verify_rfid else ''
        if verify_rfid and not (s_tag and d_tag):
            raise MissionError(f'no RFID tag registered for {s if not s_tag else d} (see rfid: in layout.yaml)')
        self.log(f'MOVE {s} -> {d}' + (f'   (RFID {s_tag} -> {d_tag})' if verify_rfid else ''))
        self.set_footprint(False)
        yaw = self.go_staging(s, 'pick')
        self.pick(s, yaw, s_tag)
        self.set_footprint(True)
        self.nav.clearAllCostmaps()
        try:
            yaw = self.go_staging(d, 'drop')
            self.place(d, yaw, tag=d_tag)
        except MissionError:
            self.log('  failed with a rack on board: it stays on the plate. Fix the problem, then use: '
                     'ros2 run arnav_mission goto <slot> / place')
            raise
        self.set_footprint(False)
        self.nav.clearAllCostmaps()
        lay.move(s, d)
        if go_home:
            self.go(*lay.home, what='home')
        self.log(f'DONE {s} -> {d}')

    def rfid_move(self, pick_tag, place_tag, go_home=False, force=False):
        """'pick 01 place 02': move the rack on the slot tagged pick_tag to the slot tagged place_tag.
        Both tags are checked with the RFID reader once the robot is under the rack / at the slot."""
        try:
            src, dst = self.tags.slot_of(pick_tag), self.tags.slot_of(place_tag)
        except TagError as e:
            raise MissionError(str(e.args[0]))
        self.move_rack(src, dst, go_home=go_home, force=force, verify_rfid=True)

    def shutdown(self):
        try:
            self.nav.destroy_node()
        finally:
            rclpy.try_shutdown()


# ====================================================================================================
# show_layout
# ====================================================================================================

def show_layout_main():
    lay = Layout()
    occ = lay.occupied()
    print(f'home: x={lay.home[0]:.1f} y={lay.home[1]:.1f}\n')
    print(f'{"slot":<10}{"x":>8}{"y":>8}   rack')
    for name, (x, y) in lay.slots.items():
        print(f'{name:<10}{x:8.2f}{y:8.2f}   {occ.get(name, "-")}')



# ====================================================================================================
# goto
# ====================================================================================================

GOTO_USAGE = '''ros2 run arnav_mission goto home | slot_05 | w1 | X Y [YAW_DEG]   (plain Nav2 goal, no rack handling)'''


def goto_main():
    a = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    if not a:
        print(GOTO_USAGE)
        sys.exit(2)
    m = Mission()
    try:
        m.wait_ready()
        if len(a) >= 2 and a[0].lstrip('-').replace('.', '', 1).isdigit():
            x, y = float(a[0]), float(a[1])
            yaw = math.radians(float(a[2])) if len(a) > 2 else 0.0
        elif a[0] == 'home':
            x, y, yaw = m.layout.home
        else:
            x, y = m.layout.resolve(a[0])
            yaw = 0.0
        m.go(x, y, yaw, what=a[0])
    except (MissionError, KeyError) as e:
        print(f'ERROR: {e}')
        m.shutdown()
        sys.exit(1)
    m.shutdown()



# ====================================================================================================
# move_rack
# ====================================================================================================

# <from>/<to>: slot_01 .. slot_10, w1..w6, or just a number (7 = slot_07)


def move_rack_main():
    ap = argparse.ArgumentParser(description='Move the rack in slot FROM to slot TO.')
    ap.add_argument('src')
    ap.add_argument('dst')
    ap.add_argument('--home', action='store_true', help='drive back to the charging bay afterwards')
    ap.add_argument('--force', action='store_true', help='do not refuse an occupied target slot (the sim will)')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    m = Mission()
    try:
        m.wait_ready()
        m.move_rack(args.src, args.dst, go_home=args.home, force=args.force)
    except (MissionError, KeyError) as e:
        print(f'ERROR: {e}')
        m.shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        m.nav.cancelTask()
    m.shutdown()



# ====================================================================================================
# run_tasks
# ====================================================================================================

def run_tasks_main():
    ap = argparse.ArgumentParser()
    ap.add_argument('file', nargs='?', default=os.path.join(share_dir('arnav_mission'), 'config', 'demo_tasks.yaml'))
    ap.add_argument('--home', action='store_true', help='go back to the charging bay when finished')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    with open(args.file) as f:
        tasks = yaml.safe_load(f)['tasks']
    m = Mission()
    done = 0
    try:
        m.wait_ready()
        for i, t in enumerate(tasks, 1):
            print(f'--- task {i}/{len(tasks)}')
            m.move_rack(t['from'], t['to'])
            done += 1
        if args.home:
            m.go(*m.layout.home, what='home')
    except (MissionError, KeyError) as e:
        print(f'ERROR on task {done + 1}: {e}')
        m.shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        m.nav.cancelTask()
    print(f'{done}/{len(tasks)} tasks done')
    m.shutdown()



# ====================================================================================================
# rfid_cmd
# ====================================================================================================

RFID_USAGE = '''Move a rack by RFID tag.

  ros2 run arnav_mission rfid_cmd --pick 01 place w1      centre rack 01 -> work station w1
  ros2 run arnav_mission rfid_cmd --pick w2 place 04      work station w2 -> centre rack position 04
  ros2 run arnav_mission rfid_cmd                         prompt: type  --pick 01 place w1  one after another
  ros2 run arnav_mission rfid_cmd --tags                  list tags and which one holds a rack

Centre racks are 01, 02, ...; work stations are w1..w3 (left) and w4..w6 (right).  Add 'home' at the end to
drive back to the charger afterwards. 
'''

def list_tags():
    lay = Layout()
    reg = TagRegistry(lay.d)
    occ = lay.occupied()
    print(f'{"tag":<6}{"where":<10}rack')
    for tag, slot in reg.tags.items():
        print(f'{tag:<6}{slot:<10}{occ.get(slot, "-")}')


def run_one(m, text):
    pick_tag, place_tag, home = parse_command(text)
    m.rfid_move(pick_tag, place_tag, go_home=home)


def rfid_cmd_main():
    words = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    if words and words[0] in ('-h', '--help'):
        print(RFID_USAGE)
        return
    if words == ['--tags']:
        list_tags()
        return
    text = ' '.join(words)
    if text:
        try:
            parse_command(text)
        except ValueError as e:
            print(f'ERROR: {e}')
            sys.exit(2)
    m = Mission()
    code = 0
    try:
        m.wait_ready()
        if text:
            run_one(m, text)
        else:
            print('type e.g.  --pick 01 place w1     ("tags" lists tags, "quit" exits)')
            while True:
                try:
                    line = input('rfid> ').strip()
                except EOFError:
                    break
                if line in ('quit', 'exit', 'q'):
                    break
                if line == 'tags':
                    list_tags()
                elif line:
                    try:
                        run_one(m, line)
                    except (MissionError, KeyError, ValueError) as e:
                        print(f'ERROR: {e}')      # stay in the prompt; the rack table is only updated on success
    except (MissionError, KeyError) as e:
        print(f'ERROR: {e}')
        code = 1
    except KeyboardInterrupt:
        m.nav.cancelTask()
    m.shutdown()
    sys.exit(code)
