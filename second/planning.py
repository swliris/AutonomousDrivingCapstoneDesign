"""Geometric lane changes, reversible bicycle Hybrid A*, and cusp-aware tracking."""
import heapq
import itertools
import math
import time
import numpy as np
from .geometry import collides, contained, footprint, inverse, path_clear, propagate, transform
from .types import Pose, Vehicle, Waypoint, wrap


def with_headings(points, direction=1):
    p = np.asarray(points, float)
    if len(p) < 2:
        return []
    tangent = np.gradient(p, axis=0)
    yaw = np.arctan2(tangent[:, 1], tangent[:, 0]) + (math.pi if direction < 0 else 0.)
    return [Waypoint(float(x), float(y), wrap(float(a)), direction) for (x, y), a in zip(p, yaw)]


def lane_change(pose, target_path, v, length=2.8):
    """Blend current heading into observed target lane in a local metric frame."""
    target = inverse([(p.x, p.y) for p in target_path], pose)
    target = target[target[:, 0] > .05]
    if len(target) < 4 or target[:, 0].max() < length:
        return []
    target = target[np.argsort(target[:, 0])]
    x = np.linspace(0., length, max(30, int(length/.04)))
    y_target = np.interp(x, target[:, 0], target[:, 1])
    t = x/length
    blend = 10*t**3-15*t**4+6*t**5
    y = y_target*blend
    dy = np.gradient(y, x)
    ddy = np.gradient(dy, x)
    curvature = np.abs(ddy)/(1+dy**2)**1.5
    if np.max(curvature) > math.tan(v.max_steer)/v.wheelbase:
        return []
    return with_headings(transform(np.column_stack([x, y]), pose))


def observed_free(path, pose, scan_angles, scan_ranges, lidar_pose, v, max_gap=.07):
    """Require actual rays beyond the swept footprint; unknown space is occupied."""
    if not path or len(scan_angles) < 20:
        return False
    angles = np.asarray(scan_angles)
    ranges = np.asarray(scan_ranges)
    for p in path[::max(1, len(path)//35)]:
        body_samples = footprint(Pose(p.x, p.y, p.yaw), v, v.margin)
        local = inverse(body_samples, pose)
        # Current body already occupies this space; ignore points inside it.
        current = ((local[:, 0] >= -v.rear) & (local[:, 0] <= v.front)
                   & (np.abs(local[:, 1]) <= v.width/2))
        sensor = inverse(local[~current], lidar_pose)
        for point in sensor:
            radius = np.linalg.norm(point)
            angle = math.atan2(point[1], point[0])
            diff = np.abs((angles-angle+math.pi) % (2*math.pi)-math.pi)
            idx = int(np.argmin(diff))
            if diff[idx] > max_gap or ranges[idx] < radius+.03:
                return False
    return True


def hybrid_astar(start: Pose, goal: Pose, vehicle: Vehicle, obstacles, allowed,
                 final_reverse=False, resolution=.10, step=.12, max_nodes=22000, timeout=2.):
    """Bounded search: no in-place turns or differential steering.

    Each edge is checked at <= 3 cm intervals. A failed/budgeted search returns
    no path and the caller remains stopped. Goal tolerance must be rechecked by
    the parking success monitor, never treated as parking success itself.
    """
    def key(p, direction):
        return (round(p.x/resolution), round(p.y/resolution), round(wrap(p.yaw)/math.radians(8)), direction)

    # Precompute convex-area half planes. Full edge sampling at every search
    # primitive made parking search too slow even in an empty rectangular bay.
    normals = offsets = None
    if not isinstance(allowed, dict):
        poly = np.asarray(allowed, float)
        edges = np.roll(poly, -1, axis=0)-poly
        nxt = np.roll(edges, -1, axis=0)
        turns = edges[:, 0]*nxt[:, 1]-edges[:, 1]*nxt[:, 0]
        if np.all(turns >= 0) or np.all(turns <= 0):
            sign = 1 if np.sum(turns) >= 0 else -1
            normals = np.column_stack([-edges[:, 1], edges[:, 0]])*sign
            offsets = np.sum(normals*poly, axis=1)

    def valid(p):
        if collides(p, obstacles, vehicle):
            return False
        shape = footprint(p, vehicle, vehicle.margin)
        if normals is not None:
            return bool(np.all(shape@normals.T-offsets >= -1e-9))
        return contained(shape, allowed)

    if not valid(start) or not valid(goal):
        return []
    start_key = key(start, 1)
    nodes = {start_key: (start, None, [], 0., 0.)}
    heap = [(0., 0, start_key)]
    counter = itertools.count(1)
    deadline = time.monotonic()+timeout
    finish = None
    while heap and len(nodes) < max_nodes and time.monotonic() < deadline:
        _, _, k = heapq.heappop(heap)
        p, parent, edge, cost, prev_steer = nodes[k]
        distance = math.hypot(p.x-goal.x, p.y-goal.y)
        local_goal = inverse([[goal.x, goal.y]], p)[0]
        straight_direction = 1 if local_goal[0] >= 0 else -1
        if abs(local_goal[1]) < .012 and abs(wrap(goal.yaw-p.yaw)) < .015 and (not final_reverse or straight_direction < 0):
            steps = max(1, int(abs(local_goal[0])/.025)+1)
            connector = [propagate(p, float(d), 0., vehicle.wheelbase) for d in np.linspace(0, local_goal[0], steps+1)[1:]]
            if all(valid(q) for q in connector):
                nk = ("analytic", next(counter))
                nodes[nk] = (connector[-1], k, [Waypoint(q.x, q.y, q.yaw, straight_direction) for q in connector], cost+distance, 0.)
                finish = nk
                break
        if distance < .07 and abs(wrap(p.yaw-goal.yaw)) < math.radians(6) and (not final_reverse or k[-1] == -1):
            finish = k
            break
        for direction in (1, -1):
            for steer in (-vehicle.max_steer, -vehicle.max_steer/2, 0., vehicle.max_steer/2, vehicle.max_steer):
                samples = [propagate(p, direction*step*t/4, steer, vehicle.wheelbase) for t in range(1, 5)]
                if not all(valid(q) for q in samples):
                    continue
                q = samples[-1]
                nk = key(q, direction)
                new_cost = cost+step*(1.3 if direction < 0 else 1.)+.06*abs(steer-prev_steer)
                if direction != k[-1]:
                    new_cost += .8
                if nk in nodes and new_cost >= nodes[nk][3]:
                    continue
                edge = [Waypoint(qi.x, qi.y, qi.yaw, direction) for qi in samples]
                nodes[nk] = (q, k, edge, new_cost, steer)
                heuristic = math.hypot(q.x-goal.x, q.y-goal.y)+.5*abs(wrap(q.yaw-goal.yaw))
                heapq.heappush(heap, (new_cost+1.6*heuristic, next(counter), nk))
    if finish is None:
        return []
    edges = []
    while nodes[finish][1] is not None:
        p, parent, edge, _, _ = nodes[finish]
        edges.append(edge)
        finish = parent
    route = [q for edge in reversed(edges) for q in edge]
    if not route:
        return []
    return [Waypoint(start.x, start.y, start.yaw, route[0].direction)]+route


class Tracker:
    def __init__(self, vehicle, lookahead=.35):
        self.vehicle = vehicle
        self.lookahead = lookahead
        self.path = []
        self.index = 0
        self.cusp_since = None
        self.cusp_pose = None

    def set_path(self, path):
        self.path = path
        self.index = 0
        self.cusp_since = None
        self.cusp_pose = None

    def update(self, pose, measured_speed, now):
        if not self.path:
            return 0., 0, True
        direction = self.path[self.index].direction
        end = self.index
        while end+1 < len(self.path) and self.path[end+1].direction == direction:
            end += 1
        candidates = range(self.index, min(end+1, self.index+60))
        self.index = min(candidates, key=lambda i: math.hypot(self.path[i].x-pose.x, self.path[i].y-pose.y))
        last = self.path[end]
        close = math.hypot(last.x-pose.x, last.y-pose.y) < .07 and abs(wrap(last.yaw-pose.yaw)) < math.radians(7)
        if close:
            if end == len(self.path)-1:
                return 0., 0, True
            stationary = abs(measured_speed) < .025
            if self.cusp_pose and (math.hypot(pose.x-self.cusp_pose.x, pose.y-self.cusp_pose.y) > .015
                                   or abs(wrap(pose.yaw-self.cusp_pose.yaw)) > .025):
                self.cusp_since = None
            if not stationary:
                self.cusp_since = None
            if self.cusp_since is None and stationary:
                self.cusp_since, self.cusp_pose = now, pose
            if self.cusp_since is not None and now-self.cusp_since >= .35:
                self.index = end+1
                self.cusp_since = None
            return 0., 0, False
        target = self.path[self.index]
        for i in range(self.index, end+1):
            target = self.path[i]
            if math.hypot(target.x-pose.x, target.y-pose.y) >= self.lookahead:
                break
        local = inverse([[target.x, target.y]], pose)[0]
        # Signed curvature works for both forward and reverse bicycle motion.
        distance2 = float(local @ local)
        steer = math.atan2(2*self.vehicle.wheelbase*local[1], max(distance2, .01))
        return float(np.clip(steer, -self.vehicle.max_steer, self.vehicle.max_steer)), direction, False
