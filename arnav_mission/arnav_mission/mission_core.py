"""
ARNAV mission package.
Sections:
  1. Warehouse layout: slots, stations, racks, staging poses, free-space checks
  2. RFID tag registry and the text command 'pick 01 place 02'
"""
import math
import os
import re

import numpy as np
import yaml


# ====================================================================================================
# 1. Layout
# ====================================================================================================

YAWS = (0.0, math.pi, math.pi / 2, -math.pi / 2)    # directions the robot can drive into a rack


def share_dir(pkg):
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(pkg)


def load_layout(path=None):
    path = path or os.path.join(share_dir('arnav_sim'), 'config', 'layout.yaml')
    with open(path) as f:
        return yaml.safe_load(f)


class GridMap:
    """The static occupancy map (pgm + yaml) for quick 'is this spot free' questions."""

    def __init__(self, yaml_path=None):
        yaml_path = yaml_path or os.path.join(share_dir('arnav_mapping'), 'maps', 'arnav_warehouse.yaml')
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        self.res = float(meta['resolution'])
        self.ox, self.oy = float(meta['origin'][0]), float(meta['origin'][1])
        with open(os.path.join(os.path.dirname(yaml_path), meta['image']), 'rb') as f:
            assert f.readline().strip() == b'P5'
            w, h = map(int, f.readline().split())
            f.readline()
            self.img = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
        self.h, self.w = self.img.shape

    def free(self, x, y, radius=0.0):
        """True if every cell within `radius` of (x, y) is known free space."""
        i = int((x - self.ox) / self.res)
        j = int((y - self.oy) / self.res)
        r = int(math.ceil(radius / self.res))
        j0, j1, i0, i1 = j - r, j + r, i - r, i + r
        if i0 < 0 or j0 < 0 or i1 >= self.w or j1 >= self.h:
            return False
        patch = self.img[self.h - 1 - j1:self.h - j0, i0:i1 + 1]
        return bool(np.all(patch >= 250))


class Layout:
    def __init__(self, data=None, grid=None):
        self.d = data or load_layout()
        self.grid = grid
        self.slots = {k: (float(v['x']), float(v['y'])) for k, v in self.d['slots'].items()}
        self.racks = dict(self.d['racks'])                      # rack -> slot
        self.stage = float(self.d.get('staging_distance', 1.5))
        h = self.d['home']
        self.home = (float(h['x']), float(h['y']), float(h['yaw']))

    def resolve(self, name):
        n = str(name).strip().lower()
        if n in ('home', 'charger'):
            return self.home[:2]
        if n.isdigit():
            n = f'slot_{int(n):02d}'
        if n not in self.slots:
            raise KeyError(f"unknown slot '{name}'. Try: ros2 run arnav_mission show_layout")
        return self.slots[n]

    def slot_name(self, name):
        n = str(name).strip().lower()
        return f'slot_{int(n):02d}' if n.isdigit() else n

    def occupied(self):
        return {s: r for r, s in self.racks.items()}

    def staging(self, centre, yaw):
        return (centre[0] - self.stage * math.cos(yaw), centre[1] - self.stage * math.sin(yaw), yaw)

    def entries(self, slot, robot_xy):
        """Approach options for a slot, best first: [(yaw, (x, y, yaw))].  Free staging spots only."""
        centre = self.resolve(slot)
        occ = {s: self.slots[s] for s in self.occupied() if s in self.slots}
        out = []
        for yaw in YAWS:
            st = self.staging(centre, yaw)
            if self.grid is not None and not self.grid.free(st[0], st[1], 0.50):
                continue
            # keep clear of other parked racks (the slot being asked for does not count)
            if any(math.hypot(st[0] - p[0], st[1] - p[1]) < 1.0 for s, p in occ.items()
                   if self.slots.get(s) != centre):
                continue
            cost = math.hypot(st[0] - robot_xy[0], st[1] - robot_xy[1]) + (3.0 if abs(math.sin(yaw)) > 0.5 else 0.0)
            out.append((cost, yaw, st))
        out.sort(key=lambda t: t[0])
        return [(y, s) for _, y, s in out]

    def move(self, src, dst):
        """Record a finished move in the in-memory rack table."""
        s, d = self.slot_name(src), self.slot_name(dst)
        for r, slot in self.racks.items():
            if slot == s:
                self.racks[r] = d
                return r
        return None


# ====================================================================================================
# 2. RFID 
# ====================================================================================================

class TagError(KeyError):
    pass


class TagRegistry:
    """Maps RFID tag ids to the slot / station each tag is stuck to (the `rfid:` block of layout.yaml)."""

    def __init__(self, layout_data):
        r = layout_data.get('rfid') or {}
        self.tags = {str(k): str(v) for k, v in (r.get('tags') or {}).items()}
        self.epc = {str(k).upper(): str(v) for k, v in (r.get('epc') or {}).items()}
        self.read_range = float(r.get('read_range', 0.15))
        self._by_slot = {v: k for k, v in self.tags.items()}

    def normalize(self, tag):
        """'1' -> '01', 'W1' -> 'w1', a known EPC -> its short id.  Raises TagError if unknown."""
        t = str(tag).strip()
        if t.upper() in self.epc:
            t = self.epc[t.upper()]
        if t.lower() in self.tags:
            return t.lower()
        if t.isdigit() and f'{int(t):02d}' in self.tags:
            return f'{int(t):02d}'
        raise TagError(f"unknown RFID tag '{tag}'. Known: {', '.join(sorted(self.tags))}")

    def slot_of(self, tag):
        return self.tags[self.normalize(tag)]

    def tag_of(self, slot):
        return self._by_slot.get(slot)


_CMD = re.compile(r'^\s*-{0,2}pick\s+(\S+)\s+-{0,2}(?:place|to)\s+(\S+)(?:\s+-{0,2}(home))?\s*$', re.IGNORECASE)


def parse_command(text):
    """'pick 01 place w1 [home]' (leading -- is optional) -> ('01', 'w1', go_home).  ValueError on anything else."""
    m = _CMD.match(text or '')
    if not m:
        raise ValueError("expected: --pick <rack or station> place <rack or station>   e.g.  --pick 01 place w1")
    return m.group(1), m.group(2), bool(m.group(3))
