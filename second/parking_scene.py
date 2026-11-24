"""Identify the gap between the two reference cars from paint and LiDAR."""
import math

import numpy as np

from .geometry import transform
from .markings import GroundView


class PaintedLines:
    """Merge overlapping observations of an aisle-aligned painted edge."""
    def __init__(self):
        self.lines = []

    def add(self, coord, low, high):
        overlaps = [line for line in self.lines if abs(line[0]-coord) < .09
                    and line[1] <= high+.18 and line[2] >= low-.18]
        for line in overlaps:
            self.lines.remove(line)
        n = sum(line[3] for line in overlaps)
        coord = (coord+sum(line[0]*line[3] for line in overlaps))/(n+1)
        self.lines.append((coord, min([low]+[l[1] for l in overlaps]),
                           max([high]+[l[2] for l in overlaps]), n+1))


class ParkingScene:
    # Order of the designated gaps, counted from IN in the direction of OUT.
    bay_order = {"perpendicular": {1: "1", 2: "2"}, "parallel": {1: "2", 2: "1"}}

    def __init__(self, config, vehicle):
        self.views = {name: GroundView(config["cameras"][name], rear=name == "rear") for name in ("front", "rear")}
        self.dividers = PaintedLines()
        self.longitudinal = PaintedLines()
        self.start_line = None
        self.vehicle = vehicle
        self.slots = {"parallel": {}, "perpendicular": {}}
        self.lips = {}
        self.seen_row = False
        self.out_line = None
        self.entry_lines = []
        self.paint_stamp = -math.inf
        self.search_limit = None

    def see_paint(self, name, mask, pose, stamp):
        view = self.views[name]
        segments = view.segments(view.project(mask))
        for local in segments:
            a, b = transform(local, pose)
            dx, dy = np.abs(b-a)
            if dx < .18*dy:
                self.dividers.add((a[0]+b[0])/2, min(a[1], b[1]), max(a[1], b[1]))
            elif dy < .18*dx:
                self.longitudinal.add((a[1]+b[1])/2, min(a[0], b[0]), max(a[0], b[0]))
        if len(segments) and name == "front":
            self.paint_stamp = stamp
            # Bound exploration by the distance to paint actually seen ahead.
            visible = transform(segments.reshape(-1, 2), pose)
            self.search_limit = float(visible[:, 0].max())

    def update(self, pose, scan, stamp):
        # The IN marking spans the aisle and is the first such line seen at departure.
        transverse = [l for l in self.dividers.lines if l[1] < -.15 and l[2] > .15 and l[2]-l[1] > 1.15 and l[3] >= 2]
        if self.start_line is None and abs(pose.x) < 1.3:
            near = [l for l in transverse if -.2 < l[0] < 1.9]
            if near:
                self.start_line = min(near)[0]

        self.entry_lines = []
        for kind in self.slots:
            self.find_row(kind, scan)
        # OUT spans the aisle beyond the parallel row. IN and bay dividers cannot qualify.
        if "parallel" in self.lips:
            _, _, end = self.lips["parallel"]
            beyond = [l for l in transverse if end+.18 < l[0] < end+2.5]
            if beyond:
                x, lo, hi, _ = min(beyond)
                self.out_line = [[x, lo], [x, hi]]

        v = self.vehicle
        right = self.lips.get("perpendicular", (None, None, None))[0]
        left = self.lips.get("parallel", (None, None, None))[0]
        # A car starting at any of the four marks is aligned with the aisle.
        # Until its lips are seen, keep a straight search strip about that heading.
        right = right if right is not None else -v.width/2-v.margin-.15
        left = left if left is not None else v.width/2+v.margin+.15
        # The physical starting position is allowed to approach the entrance.
        low = min(-v.rear-.15, pose.x-v.rear-.1)
        high = max(pose.x+2., max((end for _, _, end in self.lips.values()), default=0.))
        if self.out_line:
            high = self.out_line[0][0]+v.front
        # Inside the marked rows the reference cars occupy only part of each
        # bay. LiDAR collision and visibility checks govern those manoeuvres.
        right_edge = right-1.5 if "perpendicular" in self.lips else right
        left_edge = left+.95 if "parallel" in self.lips else left
        area = [[low, right_edge], [high, right_edge], [high, left_edge], [low, left_edge]]
        perp_area = [[low, right_edge], [high, right_edge], [high, left], [low, left]]
        aisle_y = float(np.clip(0., right+v.width/2+v.margin, left-v.width/2-v.margin))
        end = min(pose.x+1., high-v.front)
        if self.search_limit is not None:
            end = min(end, self.search_limit-v.front-v.margin)
        if self.out_line:
            end = min(end, self.out_line[0][0]-v.front-v.margin)
        elif "parallel" in self.lips:
            end = min(end, self.lips["parallel"][2])
        search = None
        aisle_visible = self.start_line is not None or self.seen_row
        if aisle_visible and stamp-self.paint_stamp < 1. and end > pose.x+.25:
            search = [end, aisle_y, 0.]
        park = dict(self.slots, area_polygon=area, perpendicular_area=perp_area, parallel_entry_lines=self.entry_lines, out_line=self.out_line,
                    out_goal=None, search=search)
        if self.out_line:
            park["out_goal"] = [self.out_line[0][0]-.5*v.wheelbase, aisle_y, 0.]
        return park

    def outside(self, tires):
        # Only classify a departure where both rows give a measured lateral extent.
        if len(self.lips) != 2:
            return False
        right, start_r, end_r = self.lips["perpendicular"]
        left, start_l, end_l = self.lips["parallel"]
        corners = np.concatenate(tires)
        if np.any((corners[:, 0] < max(start_r, start_l)) | (corners[:, 0] > min(end_r, end_l))):
            return False
        return bool(np.all(corners[:, 1] < right-1.5) or np.all(corners[:, 1] > left+.95))

    def find_row(self, kind, scan):
        perpendicular = kind == "perpendicular"
        # From IN, perpendicular bays are to the right, parallel bays to the left.
        sign = -1 if perpendicular else 1
        depth = 1.5 if perpendicular else .95
        pitch = 1.0 if perpendicular else 1.55
        cross = []
        for x, lo, hi, hits in self.dividers.lines:
            near, far = sorted((sign*lo, sign*hi))
            if near > .05 and .32*depth < far-near < 1.25*depth and hits >= 2:
                cross.append((x, near, far))
        cross.sort()
        if len(cross) < 2:
            return
        # A longitudinal painted edge joins the dividers at the back of the row.
        rows = []
        for y, a, b, _ in self.longitudinal.lines:
            back = sign*y
            matches = [line for line in cross if abs(line[2]-back) < .18
                       and line[1] < back-.5*depth and a-.20 <= line[0] <= b+.20]
            if len(matches) >= 2:
                rows.append((len(matches), b-a, back, matches))
        if not rows:
            return
        _, _, back, matches = max(rows, key=lambda row: row[:2])
        lips = [near for _, near, far in matches if abs(far-near-depth) < .15]
        if not lips:
            return
        lip = float(np.median(lips))
        if lip < .05:
            return
        # Anchor the ordinal to the entrance marking or to all five dividers.
        xs = sorted(set(x for x, _, _ in matches))
        if any(abs(b-a-pitch) < .17 for a, b in zip(xs, xs[1:])):
            self.seen_row = True
        first = None
        if len(xs) >= 5 and abs(xs[-1]-xs[0]-4*pitch) < .24:
            first = xs[0]
        elif self.start_line is not None:
            starts = [x for x in xs if .65 < x-self.start_line < 1.30]
            if len(starts) == 1:
                first = starts[0]
        if first is None and kind in self.lips:
            first = self.lips[kind][1]
        if not perpendicular:
            for a, b in zip(xs, xs[1:]):
                if abs(b-a-pitch) <= .17:
                    self.entry_lines.append([[a, lip], [b, lip]])
        if first is None:
            return
        self.lips[kind] = (sign*lip, first, first+4*pitch)
        for a, b in zip(xs, xs[1:]):
            idx = round((a-first)/pitch)
            if abs((a-first)-idx*pitch) > .18 or abs(b-a-pitch) > .17:
                continue
            number = self.bay_order[kind].get(idx)
            if number is None:
                continue
            # Reflections inside both neighbouring bays identify the rulebook's
            # reference vehicles. A bare rectangle or the outer spare bay cannot win.
            y = sign*scan[:, 1]
            in_row = (y > lip+.08) & (y < back-.06)
            x = scan[in_row, 0]
            if (np.count_nonzero((x > a-pitch+.08) & (x < a-.08)) < 3
                    or np.count_nonzero((x > b+.08) & (x < b+pitch-.08)) < 3
                    or np.count_nonzero((x > a+.08) & (x < b-.08)) >= 3):
                continue
            self.slots[kind][number] = self.slot(a, b, sign*lip, sign*back, perpendicular)

    def slot(self, a, b, lip, back, perpendicular):
        v = self.vehicle
        low, high = sorted((lip, back))
        poly = [[a+.035, low+.035], [b-.035, low+.035], [b-.035, high-.035], [a+.035, high-.035]]
        mid, y = (a+b)/2, (lip+back)/2
        shift = (v.front-v.rear)/2
        goal = [mid, y-shift, math.pi/2] if perpendicular else [mid-shift, y, 0.]
        radius = v.wheelbase/math.tan(v.max_steer)
        if perpendicular:
            # A reverse quarter-circle ends aligned with the slot before the
            # final straight. The planner still checks the complete swept path.
            approach = [mid+radius, lip+radius-v.rear, 0.]
        else:
            aisle_y = lip-v.width/2-v.margin-.12
            dy = goal[1]-aisle_y
            arc_x = math.sqrt(max(0., 4*radius*dy-dy*dy))
            approach = [goal[0]+arc_x+.10, aisle_y, 0.]
        edges = [[[a, lip], [a, back]], [[b, lip], [b, back]], [[a, back], [b, back]]]
        if not perpendicular:
            edges.append([[a, lip], [b, lip]])
        return {"goal": goal, "approach": approach, "inner_polygon": poly,
                "painted_edges": edges}
