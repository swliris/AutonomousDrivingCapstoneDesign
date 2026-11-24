"""2D point-to-point ICP against a surveyed static map; no PWM dead reckoning."""
import math
import numpy as np
from scipy.spatial import KDTree
from .geometry import inverse, transform
from .types import Pose, wrap


def _dense_neighbors(reference, points):
    squared = ((points[:, None, :] - reference[None, :, :]) ** 2).sum(axis=2)
    indices = np.argmin(squared, axis=1)
    return np.sqrt(squared[np.arange(len(points)), indices]), indices


def _neighbor_query(reference, backend):
    if backend == "dense":
        return lambda points: _dense_neighbors(reference, points)
    tree = KDTree(reference)

    def query(points):
        distances, indices = tree.query(points, k=2)
        # At an exact tie the old argmin selected the first survey point.
        tied = distances[:, 0] == distances[:, 1]
        if np.any(tied):
            distances[tied, 0], indices[tied, 0] = _dense_neighbors(reference, points[tied])
        return distances[:, 0], indices[:, 0]

    return query


class Localizer:
    def __init__(self, reference, initial, max_rmse=.06, *, backend="kdtree"):
        if backend not in ("dense", "kdtree"):
            raise ValueError(f"unknown ICP backend: {backend}")
        self.backend = backend
        self.reference = np.asarray(reference, float).reshape(-1, 2)
        self.pose = initial
        self.max_rmse = max_rmse
        self.stamp = None
        self.speed = 0.
        self.rmse = math.inf

    def update(self, points, stamp):
        p = np.asarray(points, float).reshape(-1, 2)
        if len(p) < 30 or len(self.reference) < 30 or not np.isfinite(p).all():
            return self.pose, False
        # Local map crop and evenly subsampled scan keep memory bounded.
        ref_local = inverse(self.reference, self.pose)
        ref = self.reference[np.linalg.norm(ref_local, axis=1) < 9.]
        if len(ref) < 30:
            return self.pose, False
        ref = ref[::max(1, len(ref)//1800)]
        p = p[::max(1, len(p)//300)]
        neighbors = _neighbor_query(ref, self.backend)
        estimate = self.pose
        ratio = 0.
        for _ in range(18):
            world = transform(p, estimate)
            distance, idx = neighbors(world)
            good = distance < min(.30, max(.08, float(np.quantile(distance, .65))))
            ratio = float(np.mean(good))
            if np.count_nonzero(good) < 25:
                return self.pose, False
            source, dest = world[good], ref[idx[good]]
            if np.linalg.eigvalsh(np.cov(source.T))[0] < .008:
                return self.pose, False  # a single line cannot constrain this pose
            a, b = source.mean(axis=0), dest.mean(axis=0)
            u, _, vt = np.linalg.svd((source-a).T @ (dest-b))
            rot = vt.T @ u.T
            if np.linalg.det(rot) < 0:
                vt[-1] *= -1
                rot = vt.T @ u.T
            shift = b-rot@a
            xy = rot@np.array([estimate.x, estimate.y])+shift
            angle = math.atan2(rot[1, 0], rot[0, 0])
            estimate = Pose(float(xy[0]), float(xy[1]), wrap(estimate.yaw+angle))
            self.rmse = float(np.sqrt(np.mean(np.sum((source@rot.T+shift-dest)**2, axis=1))))
            if np.linalg.norm(shift) < .0005 and abs(angle) < .0005:
                break
        dt = 0. if self.stamp is None else stamp-self.stamp
        delta = math.hypot(estimate.x-self.pose.x, estimate.y-self.pose.y)
        if self.rmse > self.max_rmse or ratio < .45 or (dt > 0 and (delta/dt > 1.8 or abs(wrap(estimate.yaw-self.pose.yaw))/dt > 2.)):
            return self.pose, False
        if dt > 0:
            self.speed = ((estimate.x-self.pose.x)*math.cos(estimate.yaw)+(estimate.y-self.pose.y)*math.sin(estimate.yaw))/dt
        self.pose, self.stamp = estimate, stamp
        return estimate, True


class ScanOdometry(Localizer):
    """Estimate displacement from overlapping scans in this run's own frame."""
    def __init__(self, max_rmse):
        super().__init__([], Pose(), max_rmse)
        self.key_pose = None

    def update(self, points, stamp):
        points = np.asarray(points, float).reshape(-1, 2)
        if len(points) < 30 or not np.isfinite(points).all():
            return self.pose, False
        if self.key_pose is None:
            if np.linalg.eigvalsh(np.cov(points.T))[0] < .008:
                return self.pose, False
            self.reference = points.copy()
            self.key_pose = self.pose
            # A second scan must confirm motion before the controller can start.
            return self.pose, False
        pose, ok = super().update(points, stamp)
        if ok and (math.hypot(pose.x-self.key_pose.x, pose.y-self.key_pose.y) > .12
                   or abs(wrap(pose.yaw-self.key_pose.yaw)) > .12):
            self.reference = transform(points, pose)
            self.key_pose = pose
        return pose, ok
