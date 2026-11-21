"""Paired timing of complete ICP updates on the same timestamped trajectory."""
import math
from time import perf_counter_ns

import numpy as np

from .geometry import inverse
from .localization import Localizer
from .types import Pose, wrap


def recorded_case(recording, config, course, mode, start_id):
    from .config import vehicle_from
    from .runtime import external_points, start_pose

    vehicle = vehicle_from(config)
    reference = np.load(course["lidar_map"], allow_pickle=False)

    def scans():
        for stamp, sensor, path in recording.events:
            if sensor == "lidar":
                raw = recording.scan(path, config)
                yield stamp, external_points(raw["points"], vehicle), None

    return reference, start_pose(course, mode, start_id), scans


def synthetic_case():
    reference = np.random.default_rng(7).uniform(-4., 4., (1800, 2))

    def scans():
        rng = np.random.default_rng(42)
        for frame in range(64):
            t = frame / 10.
            truth = Pose(.22*t, .26*math.sin(.5*t), .25*math.sin(.65*t))
            visible = reference[rng.choice(len(reference), 280, replace=False)]
            returns = inverse(visible, truth) + rng.normal(0, .004, (280, 2))
            points = np.vstack((returns, rng.uniform(-4., 4., (20, 2))))
            if frame == 32:
                points = points[:8]  # A short scan must be rejected without advancing the pose.
            yield t, points, truth

    return reference, Pose(.02, -.01, .01), scans


def compare(reference, initial, scans, max_rmse, repeats=3):
    if repeats < 1:
        raise ValueError("repeats must be positive")
    backends = ("dense", "kdtree")
    times = {name: [] for name in backends}
    accepted = dict.fromkeys(backends, 0)
    truth_errors = {name: [] for name in backends}
    pose_differences = []
    mismatches = 0
    scan_count = 0
    for repeat in range(repeats):
        localizers = {name: Localizer(reference, initial, max_rmse, backend=name) for name in backends}
        for i, (stamp, points, truth) in enumerate(scans()):
            results = {}
            order = backends if (i + repeat) % 2 == 0 else backends[::-1]
            for name in order:
                start = perf_counter_ns()
                pose, valid = localizers[name].update(points, stamp)
                times[name].append((perf_counter_ns() - start) / 1e6)
                results[name] = pose, valid
                if repeat == 0 and valid:
                    accepted[name] += 1
                    if truth is not None:
                        truth_errors[name].append(pose_error(pose, truth))
            if repeat == 0:
                scan_count += 1
                (old, old_valid), (new, new_valid) = (results[name] for name in backends)
                mismatches += old_valid != new_valid
                if old_valid and new_valid:
                    pose_differences.append(pose_error(old, new))
    if not scan_count:
        raise ValueError("no LiDAR scans to compare")
    report = {"scans": scan_count, "repeats": repeats, "validity_mismatches": mismatches,
              "both_accepted": len(pose_differences), "max_xy_delta_m": None, "max_yaw_delta_deg": None}
    if pose_differences:
        report["max_xy_delta_m"], report["max_yaw_delta_deg"] = np.max(pose_differences, axis=0).tolist()
    for name in backends:
        median, p95 = np.percentile(times[name], [50, 95]).tolist()
        stats = {"accepted": accepted[name], "median_ms": median, "p95_ms": p95}
        if truth_errors[name]:
            rms = np.sqrt(np.mean(np.square(truth_errors[name]), axis=0))
            stats["xy_rmse_m"], stats["yaw_rmse_deg"] = rms.tolist()
        report[name] = stats
    report["median_speedup"] = report["dense"]["median_ms"] / report["kdtree"]["median_ms"]
    return report


def pose_error(a, b):
    return math.hypot(a.x - b.x, a.y - b.y), abs(math.degrees(wrap(a.yaw - b.yaw)))
