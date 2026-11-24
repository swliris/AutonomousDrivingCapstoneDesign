"""Metric markings observed by calibrated cameras. All distances are metres."""
import math

import cv2
import numpy as np

from .geometry import inverse, tire_polygons, transform
from .types import Pose


class GroundView:
    """Project only the current camera's side of the vehicle onto a metric grid."""
    step = .02

    def __init__(self, camera, rear=False):
        self.top = .10 if rear else 4.50
        self.left = 3.
        scale = 1/self.step
        grid = np.array([[0., -scale, self.left*scale], [-scale, 0., self.top*scale], [0., 0., 1.]])
        self.matrix = grid @ np.asarray(camera["ground_homography"], float)
        self.size = (301, 231)
        self.image_shape = self.visible = None

    def project(self, mask):
        return cv2.warpPerspective(mask, self.matrix, self.size, flags=cv2.INTER_NEAREST)

    def visibility(self, shape):
        if self.image_shape != shape:
            self.image_shape = shape
            self.visible = self.project(np.ones(shape, np.uint8))
        return self.visible

    def segments(self, paint):
        # Long narrow edges survive even where a car occludes part of a marking.
        edges = cv2.Canny(paint, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 15, minLineLength=14, maxLineGap=4)
        if lines is None:
            return np.empty((0, 2, 2))
        uv = lines.reshape(-1, 2, 2)
        return np.stack((self.top-uv[:, :, 1]*self.step, self.left-uv[:, :, 0]*self.step), axis=2)

    def crosswalk(self, paint, lane_y, width, visible):
        # Zebra stripes run along travel, repeating laterally at the same range.
        center = round((self.left-lane_y)/self.step)
        radius = round(width*.65/self.step)
        strip = paint[:, max(0, center-radius):center+radius+1]
        if not strip.size:
            return None
        edges = np.diff(np.pad((strip != 0).astype(np.int8), ((0, 0), (1, 1))), axis=1)
        row_id, starts = np.nonzero(edges == 1)
        _, ends = np.nonzero(edges == -1)
        narrow = (ends-starts >= 2) & (ends-starts <= 9)
        centers = (starts[narrow]+ends[narrow])*.5
        row_id = row_id[narrow]
        spacing = np.diff(centers)*self.step
        pairs = (np.diff(row_id) == 0) & (spacing >= .10) & (spacing <= .30)
        indices = np.unique(row_id[1:-1][pairs[1:] & pairs[:-1]])
        for group in reversed(np.split(indices, np.flatnonzero(np.diff(indices) > 1)+1)):
            if .24 <= len(group)*self.step <= 1.25 and group[-1] < len(strip)-1:
                # A stripe cut off at the camera border has no measured near edge.
                beyond = visible[group[-1]+1:group[-1]+4, max(0, center-radius):center+radius+1]
                if beyond.size and beyond.mean() > .70:
                    return self.top-group[-1]*self.step
        return None


def road_corridor(lanes, pose, vehicle, width):
    """Bound local motion by both observed road edges and observed path length."""
    paths = [inverse([(p.x, p.y) for p in path], pose) for path in lanes.values() if path]
    paths = [p for p in paths if p[0, 0] <= .6 and p[-1, 0] > .35 and np.all(np.diff(p[:, 0]) > 0)]
    if not paths:
        return None
    ends = np.array([p[-1, 0] for p in paths])
    xs = np.unique(np.r_[-vehicle.rear-vehicle.margin-.1, np.linspace(.15, ends.max(), 45),
                         ends, np.minimum(ends+.0001, ends.max())])
    left, right = [], []
    for points in paths:
        y = np.interp(xs, *points.T)
        slope = np.gradient(points[:, 1], points[:, 0])
        y += np.minimum(xs-points[0, 0], 0.)*slope[0]
        half = .5*width*np.hypot(1., np.interp(xs, points[:, 0], slope))
        left.append(np.where(xs <= points[-1, 0], y+half, -np.inf))
        right.append(np.where(xs <= points[-1, 0], y-half, np.inf))
    ring = np.column_stack((np.r_[xs, xs[::-1]], np.r_[np.max(left, axis=0), np.min(right, axis=0)[::-1]]))
    outline = cv2.approxPolyDP(ring.astype(np.float32), .001, True).reshape(-1, 2)
    return transform(outline, pose).tolist()


def outside_road(lanes, pose, vehicle, width, steer):
    tires = np.concatenate(tire_polygons(Pose(), vehicle, steer))
    for lane, path in lanes.items():
        local = inverse([(p.x, p.y) for p in path], pose)
        x = tires[:, 0]
        visible = (x >= local[0, 0]) & (x <= local[-1, 0])
        center = np.interp(x, *local.T)
        slope = np.interp(x, local[:, 0], np.gradient(local[:, 1], local[:, 0]))
        outward = (tires[:, 1]-center)*(1 if lane == 1 else -1)
        if np.any(visible & (outward > width*.5*np.hypot(1., slope))):
            return True
    return False


def timer_posts(points, lane_paths, pose, lane_width):
    """Find two compact LiDAR returns straddling the observed two-lane road."""
    if not lane_paths or len(points) < 6:
        return None
    points = points[np.argsort(np.arctan2(points[:, 1], points[:, 0]))]
    clusters = np.split(points, np.flatnonzero(np.linalg.norm(np.diff(points, axis=0), axis=1) > .12)+1)
    posts = [g.mean(axis=0) for g in clusters if len(g) >= 3 and np.max(np.ptp(g, axis=0)) < .24]
    lanes = {k: inverse([(p.x, p.y) for p in path], pose) for k, path in lane_paths.items()}
    candidates = []
    for i, a in enumerate(posts):
        for b in posts[i+1:]:
            center = (a+b)/2
            if not .15 < center[0] < 4. or not 1.85*lane_width < np.linalg.norm(a-b) < 2*lane_width+.65:
                continue
            for lane, path in lanes.items():
                if not path[0, 0] <= center[0] <= path[-1, 0]:
                    continue
                y = np.interp(center[0], *path.T)
                slope = np.interp(center[0], path[:, 0], np.gradient(path[:, 1], path[:, 0]))
                normal = np.array([-slope, 1.])/np.hypot(1., slope)
                center_error = (center-[center[0], y]) @ normal - (.5 if lane == 2 else -.5)*lane_width
                if abs(center_error) < .30 and abs((a-b) @ normal) > .98*np.linalg.norm(a-b):
                    candidates.append((center[0], np.stack([a, b])))
                    break
    return transform(candidates[0][1], pose) if len(candidates) == 1 else None


class RoadMarks:
    def __init__(self, config, vehicle):
        self.view = GroundView(config["cameras"]["front"])
        self.width = config["perception"]["lane_width_m"]
        self.vehicle = vehicle
        self.zone_depth = config["perception"].get("stop_zone_depth_m", 1.5)
        self.pending_crossing = None
        self.crossing = self.pending_gate = self.gate = None
        self.gate_stamp = self.confirmed_stamp = -math.inf

    def see_paint(self, mask, lanes, pose):
        paths = [inverse([(p.x, p.y) for p in path], pose) for path in lanes.values() if path]
        # Crossing paint itself may interrupt the lane fit at the stop line.
        nearest = min(paths, key=lambda p: abs(np.interp(.7, *p.T)), default=None)
        y = float(np.interp(.7, *nearest.T)) if nearest is not None else 0.
        x = self.view.crosswalk(self.view.project(mask), y, self.width, self.view.visibility(mask.shape))
        if x is None:
            self.pending_crossing = None
            return
        near = transform([[x, y]], pose)[0]
        if self.pending_crossing is not None:
            along, across = inverse([near], self.pending_crossing)[0]
            if abs(along) < .15 and abs(across) < self.width and self.crossing is None:
                self.crossing = Pose(*near, pose.yaw)
        self.pending_crossing = Pose(*near, pose.yaw)

    def stop_zone(self, pose):
        if self.crossing is None:
            return None, False
        x, y = inverse([[pose.x, pose.y]], self.crossing)[0]
        if x > .25 or x < -5. or abs(y) > 1.5*self.width or math.cos(pose.yaw-self.crossing.yaw) < .8:
            self.crossing = self.pending_crossing = None
            return None, False
        front = x + self.vehicle.front
        inside = front <= .005 and x-self.vehicle.rear >= -self.zone_depth
        return max(0., -front-self.vehicle.margin), bool(inside)

    def see_posts(self, points, lanes, pose, stamp):
        line = timer_posts(points, lanes, pose, self.width)
        if line is not None and self.crossing is not None:
            if abs(inverse([line.mean(axis=0)], self.crossing)[0, 0]) < 1.5:
                line = None  # signal supports at the zebra cannot count as the lap timer
        if line is not None:
            if (self.pending_gate is not None and stamp-self.gate_stamp < .45 and
                    np.linalg.norm(line.mean(axis=0)-self.pending_gate.mean(axis=0)) < .18):
                self.gate, self.confirmed_stamp = line, stamp
            self.pending_gate, self.gate_stamp = line, stamp
        if self.gate is not None and stamp-self.confirmed_stamp > 1.5:
            self.gate = None
