"""
ARNAV rack stack:
Sections:
  1. Rack detection from a LiDAR scan 
  2. Docking steering and rack tracker
  3. Docking / retreat state machines
  4. Map-based pose refinement
  5. Front/rear scan merging
"""
from dataclasses import dataclass, field
import math

import numpy as np


# ====================================================================================================
# 1. Rack detection
# ====================================================================================================

HALF_PI = math.pi / 2.0


def wrap_pi(a):
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def fold_quarter(a):
    """Fold an angle into [-pi/4, pi/4): the symmetry of a square."""
    return (a + math.pi / 4.0) % HALF_PI - math.pi / 4.0


def rot(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


@dataclass
class DetectorParams:
    side: float = 0.90              # leg centre to leg centre (m)
    leg_size: float = 0.04          # leg cross-section (m)
    tol_side: float = 0.06          # allowed error on a leg-to-leg distance (m)
    tol_match: float = 0.07         # a predicted leg must be this close to a seen leg (m)
    max_range: float = 6.0          # ignore everything farther than this (m)
    min_range: float = 0.10
    leg_max_extent: float = 0.11    # a cluster wider than this is not a leg (m)
    gap_min: float = 0.05           # two neighbouring points further apart than this are split (m)
    gap_per_range: float = 2.5      # ... or further than this many beam spacings at that range
    min_points_near: int = 2        # points a leg needs when it is closer than `near_range`
    near_range: float = 4.0
    free_margin: float = 0.15       # a ray that passes a predicted leg by more than this saw free space
    dedup_dist: float = 0.40        # two racks closer than this are the same rack


@dataclass
class Rack:
    x: float
    y: float
    yaw: float                      # folded into [-pi/4, pi/4)
    legs: int                       # 2, 3 or 4 legs matched
    residual: float                 # mean distance between predicted and seen legs (m)
    confidence: float
    leg_xy: list = field(default_factory=list)   # the seen legs that were matched

    def corners(self, side):
        h = side / 2.0
        r = rot(self.yaw)
        return [np.array([self.x, self.y]) + r @ np.array(o)
                for o in ((h, h), (-h, h), (-h, -h), (h, -h))]


class ScanView:
    """Random access to a scan by bearing, used to ask 'does the ray towards a point see free space?'"""

    def __init__(self, ranges, angle_min, angle_inc, range_max):
        self.r = np.asarray(ranges, dtype=float)
        self.a0 = float(angle_min)
        self.da = float(angle_inc)
        self.rmax = float(range_max)
        self.n = len(self.r)
        # a full 360 degree scan wraps around
        self.full = abs(self.n * self.da - 2.0 * math.pi) < 3.0 * abs(self.da)

    def _index(self, bearing):
        k = (wrap_pi(bearing - self.a0)) / self.da if self.full else (bearing - self.a0) / self.da
        k = int(round(k))
        if self.full:
            return k % self.n
        return k if 0 <= k < self.n else None

    def check(self, point, margin, width=0.05):
        """Classify what the scan says about a spot where a rack leg should stand.

        'present'  - there is a return at about that distance
        'occluded' - something nearer blocks the view, or the spot is outside the scan: unknown
        'free'     - the ray passes the spot and hits something farther away (or nothing)
        """
        d = float(math.hypot(point[0], point[1]))
        if d < 1e-3:
            return 'occluded'
        bearing = math.atan2(point[1], point[0])
        k = self._index(bearing)
        if k is None:
            return 'occluded'
        # look at the beams that straddle the leg
        half = max(1, int(math.ceil(math.atan2(width, d) / abs(self.da))))
        ks = [(k + j) % self.n if self.full else k + j for j in range(-half, half + 1)]
        ks = [j for j in ks if 0 <= j < self.n]
        if not ks:
            return 'occluded'
        vals = self.r[ks]
        vals = np.where(np.isfinite(vals), vals, np.inf)
        vals = np.where(vals > self.rmax, np.inf, vals)
        if np.any(np.abs(vals - d) <= margin):
            return 'present'
        if np.all(vals > d + margin):
            return 'free'
        return 'occluded'


def scan_points(ranges, angle_min, angle_inc, range_min, range_max):
    """Valid beams as (points Nx2, ranges N, bearings N)."""
    r = np.asarray(ranges, dtype=float)
    ang = angle_min + np.arange(len(r)) * angle_inc
    ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    r, ang = r[ok], ang[ok]
    pts = np.stack([r * np.cos(ang), r * np.sin(ang)], axis=1) if len(r) else np.zeros((0, 2))
    return pts, r, ang


def find_legs(pts, rng, ang, angle_inc, p):
    """Cluster neighbouring points and return the centres of the leg-sized clusters (Kx2)."""
    n = len(pts)
    if n == 0:
        return np.zeros((0, 2))
    legs = []
    start = 0
    inc = abs(angle_inc)

    def close(i):
        """close cluster pts[start:i+1]"""
        seg = pts[start:i + 1]
        cnt = len(seg)
        r_mid = float(np.mean(rng[start:i + 1]))
        need = p.min_points_near if r_mid < p.near_range else 1
        if cnt < need:
            return
        extent = float(np.linalg.norm(seg[-1] - seg[0])) if cnt > 1 else 0.0
        if extent > p.leg_max_extent:
            return
        # a leg this far away can not make more points than its width allows
        if cnt > p.leg_max_extent / max(r_mid * inc, 1e-6) + 3:
            return
        c = seg.mean(axis=0)
        d = float(np.hypot(c[0], c[1]))
        if d < 1e-6:
            return
        # the scan only sees the near face; the leg centre lies about half a leg further out
        c = c + c / d * (p.leg_size * 0.55)
        legs.append(c)

    for i in range(1, n):
        dtheta = max(abs(ang[i] - ang[i - 1]), inc)
        thr = max(p.gap_min, p.gap_per_range * min(rng[i], rng[i - 1]) * dtheta)
        gap_far = dtheta > 3.0 * inc            # missing beams between the two points
        jump = float(np.linalg.norm(pts[i] - pts[i - 1]))
        if jump > thr or gap_far:
            close(i - 1)
            start = i
    close(n - 1)
    return np.array(legs) if legs else np.zeros((0, 2))


def _kabsch_yaw(offsets, measured):
    """Best rotation angle taking `offsets` onto `measured` (both Nx2, N >= 2)."""
    o = offsets - offsets.mean(axis=0)
    m = measured - measured.mean(axis=0)
    num = float(np.sum(o[:, 0] * m[:, 1] - o[:, 1] * m[:, 0]))
    den = float(np.sum(o[:, 0] * m[:, 0] + o[:, 1] * m[:, 1]))
    return math.atan2(num, den)


def match_racks(legs, p, view=None):
    """Find all racks in a set of leg centres.  Returns a list of Rack, best first."""
    k = len(legs)
    if k < 2:
        return []
    h = p.side / 2.0
    offsets = np.array([(h, h), (-h, h), (-h, -h), (h, -h)])
    diag = p.side * math.sqrt(2.0)
    hyps = []   # (centre, yaw)
    for i in range(k):
        for j in range(i + 1, k):
            v = legs[j] - legs[i]
            d = float(np.hypot(v[0], v[1]))
            if abs(d - p.side) <= p.tol_side:
                u = v / d
                nrm = np.array([-u[1], u[0]])
                mid = (legs[i] + legs[j]) / 2.0
                yaw = fold_quarter(math.atan2(v[1], v[0]))
                hyps.append((mid + nrm * h, yaw))
                hyps.append((mid - nrm * h, yaw))
            elif abs(d - diag) <= 1.5 * p.tol_side:
                mid = (legs[i] + legs[j]) / 2.0
                hyps.append((mid, fold_quarter(math.atan2(v[1], v[0]) - math.pi / 4.0)))

    found = []
    for centre, yaw in hyps:
        r = rot(yaw)
        corners = [centre + r @ o for o in offsets]
        used = set()
        pairs = []
        for ci, c in enumerate(corners):
            dist = np.hypot(legs[:, 0] - c[0], legs[:, 1] - c[1])
            order = np.argsort(dist)
            for li in order[:2]:
                if dist[li] <= p.tol_match and li not in used:
                    used.add(int(li))
                    pairs.append((ci, int(li), float(dist[li])))
                    break
        if len(pairs) < 2:
            continue
        cis = [a for a, _, _ in pairs]
        lis = [b for _, b, _ in pairs]
        # refine the pose from the legs that were actually seen
        meas = legs[lis]
        offs = offsets[cis]
        yaw_r = _kabsch_yaw(offs, meas)
        if abs(wrap_pi(yaw_r - yaw)) > 0.15:      # only trust the fit close to the hypothesis
            yaw_r = yaw
        rr = rot(yaw_r)
        centre_r = np.mean(meas - (rr @ offs.T).T, axis=0)
        yaw_f = fold_quarter(yaw_r)               # same square, so the centre does not change
        rr = rot(yaw_f)
        cis_set = set(cis)
        residual = float(np.mean([dd for _, _, dd in pairs]))

        legs_n = len(pairs)
        if legs_n < 4 and view is not None:
            bad = False
            pred = [centre_r + rot(yaw_r) @ o for o in offsets]
            for ci, c in enumerate(pred):
                if ci in cis_set:
                    continue
                if math.hypot(c[0], c[1]) > p.max_range + 0.5:
                    continue
                if view.check(c, p.free_margin) == 'free':
                    bad = True
                    break
            if bad:
                continue
        conf = {2: 0.5, 3: 0.8, 4: 1.0}.get(legs_n, 0.3) * max(0.5, 1.0 - residual / max(p.tol_match, 1e-6) * 0.5)
        found.append(Rack(float(centre_r[0]), float(centre_r[1]), float(yaw_f), legs_n, residual,
                          float(conf), [tuple(m) for m in meas]))

    # best first: more legs, smaller residual
    found.sort(key=lambda f: (-f.legs, f.residual))
    kept = []
    for f in found:
        dup = False
        for g in kept:
            if math.hypot(f.x - g.x, f.y - g.y) < p.dedup_dist:
                dup = True
                break
            shared = sum(1 for a in f.leg_xy for b in g.leg_xy if math.hypot(a[0] - b[0], a[1] - b[1]) < 0.03)
            if shared >= 2:
                dup = True
                break
        if not dup:
            kept.append(f)
    return kept


def detect_racks(ranges, angle_min, angle_inc, range_min, range_max, p=None):
    """Full pipeline: scan arrays in, list of Rack out (nearest first)."""
    p = p or DetectorParams()
    pts, rng, ang = scan_points(ranges, angle_min, angle_inc, max(range_min, p.min_range),
                                min(range_max, p.max_range))
    legs = find_legs(pts, rng, ang, angle_inc, p)
    view = ScanView(ranges, angle_min, angle_inc, range_max)
    racks = match_racks(legs, p, view)
    racks.sort(key=lambda f: math.hypot(f.x, f.y))
    return racks, legs


# ====================================================================================================
# 2. Docking control 
# ====================================================================================================

def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def axis_errors(robot, centre, axis_yaw):
    """robot = (x, y, yaw) and centre = (x, y) in one frame; returns (ex, e, psi)."""
    rx, ry, ryaw = robot
    dx, dy = centre[0] - rx, centre[1] - ry
    c, s = math.cos(axis_yaw), math.sin(axis_yaw)
    ex = dx * c + dy * s
    e = -(-dx * s + dy * c)          # robot lateral offset (left of the axis = +)
    psi = wrap_pi(ryaw - axis_yaw)
    return ex, e, psi


@dataclass
class AxisParams:
    v_max: float = 0.25             # m/s while approaching
    v_min: float = 0.04             # m/s creep so the wheels do not stall
    v_gain: float = 0.8             # 1/s: speed = v_gain * remaining distance
    w_max: float = 0.6              # rad/s
    k_psi: float = 2.0              # heading gain
    k_e: float = 2.0                # lateral gain (Stanley)
    k_soft: float = 0.08            # m/s, keeps the Stanley term finite at standstill
    psi_d_max: float = 0.30         # largest heading offset used to correct a lateral error (rad)
    turn_in_place_psi: float = 0.35  # above this heading error the robot rotates before it drives
    pos_tol: float = 0.015          # arrived when |ex| is below this (m)
    lat_tol: float = 0.03           # ... and |e| below this (m)
    head_tol: float = 0.05          # ... and |psi| below this (rad)
    err_slow_e: float = 0.15        # lateral error that slows the robot to a crawl (m)
    err_slow_psi: float = 0.30      # heading error that slows the robot to a crawl (rad)
    reverse_v: float = 0.20         # m/s when reversing out


class AxisController:
    def __init__(self, params=None):
        self.p = params or AxisParams()

    def arrived(self, ex, e, psi):
        p = self.p
        return abs(ex) <= p.pos_tol and abs(e) <= p.lat_tol and abs(psi) <= p.head_tol

    def _steer(self, v, e, psi):
        p = self.p
        psi_d = math.atan2(p.k_e * e, abs(v) + p.k_soft)
        psi_d = clamp(psi_d, -p.psi_d_max, p.psi_d_max)
        psi_d = -psi_d if v >= 0.0 else psi_d       # when reversing the sign of the correction flips
        w = -p.k_psi * (psi - psi_d)
        return clamp(w, -p.w_max, p.w_max)

    def step(self, ex, e, psi):
        """Forward approach.  Returns (v, w).  ex < 0 means the robot went past: it creeps back."""
        p = self.p
        if self.arrived(ex, e, psi):
            return 0.0, 0.0
        # far from straight: rotate on the spot first
        if abs(psi) > p.turn_in_place_psi and ex > 0.3:
            w = clamp(-p.k_psi * psi, -p.w_max, p.w_max)
            if abs(w) < 0.12:
                w = math.copysign(0.12, w)
            return 0.0, w
        if ex < 0.0:                                 # overshoot: back up slowly
            v = -clamp(p.v_gain * abs(ex), p.v_min, 0.10)
            return v, self._steer(v, e, psi)
        v = clamp(p.v_gain * ex, p.v_min, p.v_max)
        # large errors: slow down, but never to zero while there is still a way to correct them
        slow = 1.0 - 0.8 * clamp(max(abs(e) / p.err_slow_e, abs(psi) / p.err_slow_psi), 0.0, 1.0)
        v = max(p.v_min, v * slow)
        if ex <= 0.0 + p.pos_tol:
            # on top of the centre: only trim the heading and the lateral error, do not drive
            if abs(e) <= p.lat_tol and abs(psi) > p.head_tol:
                return 0.0, clamp(-p.k_psi * psi, -p.w_max, p.w_max)
        return v, self._steer(v, e, psi)

    def reverse(self, e, psi):
        """Straight back out along the axis at reverse_v.  Returns (v, w)."""
        v = -self.p.reverse_v
        return v, self._steer(v, e, psi)


def fold_quarter_about(angle, ref):
    """`angle` is only known modulo pi/2.  Return the equivalent angle closest to `ref`."""
    d = (angle - ref + math.pi / 4.0) % (math.pi / 2.0) - math.pi / 4.0
    return ref + d


class RackTracker:
    """Keeps one running estimate of a rack's centre and axis in a fixed (odom) frame."""

    def __init__(self, hint_x, hint_y, hint_yaw, gate=0.6, alpha=0.5, track_gate=0.30):
        self.hint = (hint_x, hint_y, hint_yaw)
        self.gate = gate
        self.track_gate = track_gate
        self.alpha = alpha
        self.x = hint_x
        self.y = hint_y
        self.phi = hint_yaw
        self.initialised = False
        self.updates = 0
        self.last_legs = 0

    def update(self, detections):
        """detections: iterable of (x, y, yaw, legs) already in the tracking frame.  Returns True if used."""
        best = None
        best_d = None
        ref = (self.x, self.y) if self.initialised else (self.hint[0], self.hint[1])
        gate = self.track_gate if self.initialised else self.gate
        for (x, y, yaw, legs) in detections:
            d = math.hypot(x - ref[0], y - ref[1])
            if d <= gate and (best is None or d < best_d):
                best, best_d = (x, y, yaw, legs), d
        if best is None:
            return False
        x, y, yaw, legs = best
        phi_meas = fold_quarter_about(yaw, self.hint[2])
        if not self.initialised:
            self.x, self.y, self.phi = x, y, phi_meas
            self.initialised = True
        else:
            # 2-leg detections are less certain than 3 or 4 legs
            a = self.alpha if legs >= 3 else self.alpha * 0.5
            self.x += a * (x - self.x)
            self.y += a * (y - self.y)
            self.phi += a * wrap_pi(phi_meas - self.phi)
        self.updates += 1
        self.last_legs = legs
        return True


# ====================================================================================================
# 3. Docking sessions (was dock_session.py)
# ====================================================================================================

SEARCHING = 'SEARCHING'
APPROACHING = 'APPROACHING'
BACKING_OFF = 'BACKING_OFF'
RETREATING = 'RETREATING'


@dataclass
class SessionParams:
    search_timeout: float = 8.0     # s to wait for the first detection
    timeout: float = 90.0           # s for the whole approach
    entry_gate_ex: float = 0.95     # m: the nose is about to pass the first pair of legs
    entry_e_max: float = 0.07       # m: lateral error allowed at the gate
    entry_psi_max: float = 0.10     # rad: heading error allowed at the gate
    backoff_to_ex: float = 1.35     # m: reverse until the rack centre is this far ahead again
    max_retries: int = 2
    stall_time: float = 20.0        # s without getting closer = give up
    stall_progress: float = 0.03    # m


@dataclass
class Command:
    v: float = 0.0
    w: float = 0.0
    state: str = SEARCHING
    done: bool = False
    error: str = ''
    ex: float = 0.0
    e: float = 0.0
    psi: float = 0.0


class ApproachSession:
    """Drive in along an axis until the robot centre is on the target centre.

    Give it a RackTracker to find the rack with the LiDAR, or `static_target=(x, y, yaw)` to drive to a
    known spot (used for slots).
    """

    def __init__(self, tracker=None, static_target=None, axis_params=None, params=None, t0=0.0):
        if (tracker is None) == (static_target is None):
            raise ValueError('give exactly one of tracker or static_target')
        self.tracker = tracker
        self.static = static_target
        self.ctrl = AxisController(axis_params or AxisParams())
        self.p = params or SessionParams()
        self.state = SEARCHING if tracker is not None else APPROACHING
        self.t0 = t0
        self.retries = 0
        self.best_ex = None
        self.best_t = t0

    def update_detections(self, detections):
        if self.tracker is not None:
            return self.tracker.update(detections)
        return False

    def target(self):
        if self.static is not None:
            return self.static
        return (self.tracker.x, self.tracker.y, self.tracker.phi)

    def step(self, robot, t):
        """robot = (x, y, yaw) in the tracking frame; t = time in seconds."""
        p = self.p
        out = Command(state=self.state)
        if self.state == SEARCHING:
            if self.tracker.initialised:
                self.state = APPROACHING
                self.best_t = t
            elif t - self.t0 > p.search_timeout:
                out.error = 'no rack found near the expected position'
                return out
            else:
                return out
        if t - self.t0 > p.timeout:
            out.error = 'docking timed out'
            return out

        tx, ty, tphi = self.target()
        ex, e, psi = axis_errors(robot, (tx, ty), tphi)
        out.ex, out.e, out.psi = ex, e, psi

        if self.state == BACKING_OFF:
            out.state = BACKING_OFF
            if ex >= p.backoff_to_ex:
                self.state = APPROACHING
                self.best_ex, self.best_t = None, t
                out.state = APPROACHING
                return out
            out.v, out.w = self.ctrl.reverse(e, psi)
            return out

        out.state = APPROACHING
        if self.ctrl.arrived(ex, e, psi):
            out.done = True
            return out

        # about to push the nose between the first two legs: be straight, or back off and try again
        if 0.7 < ex <= p.entry_gate_ex and (abs(e) > p.entry_e_max or abs(psi) > p.entry_psi_max):
            self.retries += 1
            if self.retries > p.max_retries:
                out.error = (f'could not line up with the rack (lateral {e:+.2f} m, '
                             f'heading {math.degrees(psi):+.1f} deg)')
                return out
            self.state = BACKING_OFF
            out.state = BACKING_OFF
            out.v, out.w = self.ctrl.reverse(e, psi)
            return out

        # stall watchdog
        if self.best_ex is None or ex < self.best_ex - p.stall_progress:
            self.best_ex, self.best_t = ex, t
        elif t - self.best_t > p.stall_time:
            out.error = 'no progress towards the rack'
            return out

        out.v, out.w = self.ctrl.step(ex, e, psi)
        return out


class RetreatSession:
    """Reverse straight back along the axis until the robot centre is `distance` behind the target
    (behind = on the side it drove in from, so ex = distance still to go to the target centre)."""

    def __init__(self, target, distance, axis_params=None, timeout=40.0, t0=0.0):
        self.target = target
        self.distance = distance
        self.ctrl = AxisController(axis_params or AxisParams())
        self.timeout = timeout
        self.t0 = t0

    def step(self, robot, t):
        tx, ty, tphi = self.target
        ex, e, psi = axis_errors(robot, (tx, ty), tphi)
        out = Command(state=RETREATING, ex=ex, e=e, psi=psi)
        if ex >= self.distance:
            out.done = True
            return out
        if t - self.t0 > self.timeout:
            out.error = 'retreat timed out'
            return out
        out.v, out.w = self.ctrl.reverse(e, psi)
        return out


# ====================================================================================================
# 4. Map pose refinement 
# ====================================================================================================

class MapRefiner:
    def __init__(self, data, width, height, res, ox, oy, occ_thresh=65, cap=0.60, sigma=0.08):
        """data: occupancy values (row-major, row 0 = bottom, like nav_msgs/OccupancyGrid.data)."""
        self.res, self.ox, self.oy = float(res), float(ox), float(oy)
        self.w, self.h = int(width), int(height)
        occ = (np.asarray(data, dtype=np.int16).reshape(self.h, self.w) >= occ_thresh)
        self.cap, self.sigma = float(cap), float(sigma)
        self.dist = self._distance_field(occ, self.cap, self.res)       # [j, i] metres to nearest occupied cell
        self.n_occ = int(occ.sum())

    @staticmethod
    def _distance_field(occ, cap, res):
        """Chamfer distance to the nearest occupied cell, capped at `cap` (no scipy needed)."""
        big = cap + 1.0
        d = np.where(occ, 0.0, big).astype(np.float32)
        diag = res * math.sqrt(2.0)
        for _ in range(int(math.ceil(cap / res)) + 1):
            n = d.copy()
            for axis in (0, 1):
                for sh in (1, -1):
                    s = np.roll(d, sh, axis=axis) + res
                    n = np.minimum(n, s)
            for sy in (1, -1):
                for sx in (1, -1):
                    s = np.roll(np.roll(d, sy, axis=0), sx, axis=1) + diag
                    n = np.minimum(n, s)
            if np.array_equal(n, d):
                break
            d = n
        return np.minimum(d, cap)

    # ------------------------------------------------------------------------------------------
    def _lookup(self, x, y):
        i = np.floor((x - self.ox) / self.res).astype(int)
        j = np.floor((y - self.oy) / self.res).astype(int)
        ok = (i >= 0) & (i < self.w) & (j >= 0) & (j < self.h)
        d = np.full(x.shape, self.cap, dtype=np.float32)
        d[ok] = self.dist[j[ok], i[ok]]
        return d

    def _score(self, pts, x, y, yaw):
        """Match score of the scan points (robot frame) for robot pose arrays x, y, yaw (any shape, broadcast)."""
        c, s = np.cos(yaw)[..., None], np.sin(yaw)[..., None]
        wx = x[..., None] + c * pts[:, 0] - s * pts[:, 1]
        wy = y[..., None] + s * pts[:, 0] + c * pts[:, 1]
        d = self._lookup(wx, wy)
        return np.exp(-0.5 * (d / self.sigma) ** 2).sum(axis=-1)

    def refine(self, pose, ranges, angle_min, angle_inc, rmin=0.30, rmax=11.0,
               search_xy=0.50, search_yaw=0.15, min_points=25, min_matched=0.25):
        """pose = (x, y, yaw) of the robot centre in the map.  Returns (new_pose, info dict).

        The scan must be expressed at the robot centre (the merged /scan is).  If the match is weak the input
        pose is returned unchanged and info['ok'] is False.
        """
        r = np.asarray(ranges, dtype=float)
        ang = angle_min + np.arange(len(r)) * angle_inc
        good = np.isfinite(r) & (r >= rmin) & (r <= rmax)
        pts = np.stack([r[good] * np.cos(ang[good]), r[good] * np.sin(ang[good])], axis=1)
        info = {'ok': False, 'points': int(len(pts)), 'dx': 0.0, 'dy': 0.0, 'dyaw': 0.0, 'matched': 0.0}
        if len(pts) < min_points:
            info['reason'] = f'only {len(pts)} usable scan points'
            return tuple(pose), info
        x0, y0, th0 = (float(v) for v in pose)
        n = len(pts)
        # small prior: stay at the AMCL pose unless the scan clearly says otherwise
        lam_xy = 0.02 * n / search_xy ** 2
        lam_yaw = 0.02 * n / search_yaw ** 2

        def best_in(dxs, dys, dths):
            DX, DY, DT = np.meshgrid(dxs, dys, dths, indexing='ij')
            sc = np.empty(DX.shape, dtype=np.float32)
            for k in range(DX.shape[2]):                       # one yaw at a time keeps memory small
                sc[:, :, k] = self._score(pts, x0 + DX[:, :, k], y0 + DY[:, :, k], th0 + DT[:, :, k])
            sc = sc - lam_xy * (DX ** 2 + DY ** 2) - lam_yaw * DT ** 2
            idx = np.unravel_index(int(np.argmax(sc)), sc.shape)
            return float(DX[idx]), float(DY[idx]), float(DT[idx]), float(sc[idx])

        step = 0.05
        axes = np.arange(-search_xy, search_xy + 1e-9, step)
        dx, dy, dth, _ = best_in(axes, axes, np.arange(-search_yaw, search_yaw + 1e-9, 0.025))
        for half, st, sy in ((0.05, 0.01, 0.02), (0.01, 0.0025, 0.005)):       # two finer passes
            ax = np.arange(-half, half + 1e-9, st)
            ddx, ddy, ddt, _ = best_in(dx + ax, dy + ax, dth + np.arange(-sy, sy + 1e-9, sy / 4.0))
            dx, dy, dth = ddx, ddy, ddt
        c, s = math.cos(th0 + dth), math.sin(th0 + dth)
        wx = x0 + dx + c * pts[:, 0] - s * pts[:, 1]
        wy = y0 + dy + s * pts[:, 0] + c * pts[:, 1]
        matched = float(np.mean(self._lookup(wx, wy) < 0.10))
        info.update(dx=dx, dy=dy, dyaw=dth, matched=matched)
        if matched < min_matched:
            info['reason'] = f'only {matched * 100:.0f}% of the scan matches the map'
            return (x0, y0, th0), info
        info['ok'] = True
        return (x0 + dx, y0 + dy, th0 + dth), info


# ====================================================================================================
# 5. Scan merging 
# ====================================================================================================

def beams_to_xy(ranges, angle_min, angle_inc, range_min, range_max, mount):
    """Valid beams of one lidar as points in the robot centre frame.  mount = (x, y, yaw)."""
    r = np.asarray(ranges, dtype=float)
    ang = angle_min + np.arange(len(r)) * angle_inc
    ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    r, ang = r[ok], ang[ok]
    px, py, pyaw = mount
    c, s = math.cos(pyaw), math.sin(pyaw)
    x = r * np.cos(ang)
    y = r * np.sin(ang)
    return np.stack([px + c * x - s * y, py + s * x + c * y], axis=1)


def inside_box(pts, half_x, half_y):
    return (np.abs(pts[:, 0]) <= half_x) & (np.abs(pts[:, 1]) <= half_y)


def merge(point_sets, num_bins=720, range_max=12.0, self_box=None, rack_box=None, carrying=False):
    """point_sets: list of Nx2 arrays in the centre frame.  Returns (ranges, angle_min, angle_inc).

    self_box / rack_box = (half_x, half_y).  Points inside self_box are always dropped; points inside
    rack_box are dropped only while `carrying` is true.
    """
    out = np.full(num_bins, np.inf)
    inc = 2.0 * math.pi / num_bins
    for pts in point_sets:
        if len(pts) == 0:
            continue
        keep = np.ones(len(pts), dtype=bool)
        if self_box is not None:
            keep &= ~inside_box(pts, *self_box)
        if carrying and rack_box is not None:
            keep &= ~inside_box(pts, *rack_box)
        pts = pts[keep]
        if len(pts) == 0:
            continue
        rng = np.hypot(pts[:, 0], pts[:, 1])
        ang = np.arctan2(pts[:, 1], pts[:, 0])
        idx = np.floor((ang + math.pi) / inc).astype(int) % num_bins
        np.minimum.at(out, idx, rng)
    out[out > range_max] = np.inf
    return out, -math.pi, inc
