"""Read the asynchronous image and scan files produced by runtime.record."""
import json
import math
from itertools import groupby
from operator import itemgetter
from pathlib import Path

import cv2
import numpy as np

from .config import load_json
from .sensors import Sample, vehicle_scan


class Recording:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.config = load_json(self.directory / "sensor_config.json")
        self.events = []
        with (self.directory / "frames.jsonl").open(encoding="utf-8") as stream:
            for line_no, line in enumerate(stream, 1):
                try:
                    row = json.loads(line)
                    sensor, stamp = row["sensor"], float(row["stamp"])
                    if sensor not in ("front", "rear", "lidar") or not math.isfinite(stamp):
                        raise ValueError("invalid sensor or timestamp")
                    self.events.append((stamp, sensor, self.directory / row["path"]))
                except (ValueError, KeyError, TypeError) as exc:
                    raise ValueError(f"frames.jsonl line {line_no}: {exc}") from exc
        if not self.events:
            raise ValueError("empty sensor recording")
        # The recorder polls each worker separately. File order can differ from capture order.
        self.events.sort(key=itemgetter(0))

    def scan(self, path, config):
        with np.load(path, allow_pickle=False) as data:
            points, angles, ranges = (data[key] for key in ("points", "angles", "ranges"))
        if (ranges.ndim != 1 or angles.shape != ranges.shape or points.shape != (len(ranges), 2)
                or not all(np.isfinite(a).all() for a in (points, angles, ranges))):
            raise ValueError(f"invalid LiDAR arrays in {path}")
        recorded = self.config["lidar"]
        chosen = config["lidar"]
        if (chosen["min_range_m"], chosen["max_range_m"]) != (recorded["min_range_m"], recorded["max_range_m"]):
            keep = (ranges >= chosen["min_range_m"]) & (ranges <= chosen["max_range_m"])
            points, angles, ranges = points[keep], angles[keep], ranges[keep]
        if recorded["angle_sign"] != chosen["angle_sign"] or recorded["pose"] != chosen["pose"]:
            angles = angles * (chosen["angle_sign"] / recorded["angle_sign"])
            return vehicle_scan(angles, ranges, chosen["pose"])
        return {"points": points, "angles": angles, "ranges": ranges}

    def snapshots(self, config):
        latest = dict(front=None, rear=None, lidar=None)
        for stamp, entries in groupby(self.events, key=itemgetter(0)):
            for _, sensor, path in entries:
                if sensor == "lidar":
                    data = self.scan(path, config)
                else:
                    data = cv2.imread(str(path), cv2.IMREAD_COLOR)
                    if data is None:
                        raise ValueError(f"unreadable {sensor} image: {path}")
                    camera = config["cameras"][sensor]
                    if data.shape[:2] != (camera["height"], camera["width"]):
                        raise ValueError(f"{sensor} image resolution differs from calibration: {path}")
                latest[sensor] = Sample(stamp, data)
            yield stamp, latest["front"], latest["rear"], latest["lidar"]
