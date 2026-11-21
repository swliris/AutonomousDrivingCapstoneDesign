import logging
import threading
import time
from dataclasses import dataclass
import cv2
import numpy as np
from .geometry import transform
from .types import Pose

LOG = logging.getLogger(__name__)


@dataclass
class Sample:
    stamp: float
    data: object


class Worker:
    def __init__(self):
        self.sample = None
        self.error = None
        self.lock = threading.Lock()
        self.closed = threading.Event()
        self.thread = None

    def publish(self, data):
        with self.lock:
            self.sample = Sample(time.monotonic(), data)

    def latest(self):
        with self.lock:
            return self.sample

    def start(self):
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        try:
            self.capture()
        except Exception as exc:
            self.error = str(exc)
            LOG.exception("sensor stopped")

    def close(self):
        self.closed.set()
        if self.thread:
            self.thread.join(timeout=2.)


class Camera(Worker):
    def __init__(self, config):
        super().__init__()
        self.config = config

    def capture(self):
        c = self.config
        cap = cv2.VideoCapture(c["device"], cv2.CAP_V4L2)
        try:
            if not cap.isOpened():
                raise RuntimeError(f"camera cannot open: {c['device']}")
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, c["width"])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, c["height"])
            cap.set(cv2.CAP_PROP_FPS, c["fps"])
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            while not self.closed.is_set():
                ok, frame = cap.read()
                if not ok:
                    raise RuntimeError(f"camera read failed: {c['device']}")
                if frame.shape[:2] != (c["height"], c["width"]):
                    raise RuntimeError("camera resolution differs from calibration")
                self.publish(frame)
        finally:
            cap.release()


def vehicle_scan(angles, ranges, mount):
    sensor_points = np.column_stack([ranges * np.cos(angles), ranges * np.sin(angles)])
    return {"points": transform(sensor_points, Pose(*mount)), "angles": angles, "ranges": ranges}


class Lidar(Worker):
    def __init__(self, config):
        super().__init__()
        self.config = config

    def capture(self):
        from rplidar import RPLidar
        c = self.config
        lidar = RPLidar(c["port"], baudrate=c["baud"], timeout=1.)
        try:
            LOG.info("LiDAR info=%s health=%s", lidar.get_info(), lidar.get_health())
            for scan in lidar.iter_scans(max_buf_meas=6000, min_len=30):
                if self.closed.is_set():
                    break
                scan = np.asarray(scan, float).reshape(-1, 3)
                good = (scan[:, 0] > 0) & np.isfinite(scan[:, 2]) & (scan[:, 2] >= c["min_range_m"]*1000) & (scan[:, 2] <= c["max_range_m"]*1000)
                scan = scan[good]
                angles = np.deg2rad(c["angle_sign"]*scan[:, 1])
                ranges = scan[:, 2]/1000.
                self.publish(vehicle_scan(angles, ranges, c["pose"]))
        finally:
            try:
                lidar.stop()
                lidar.stop_motor()
            finally:
                lidar.disconnect()


class SensorHub:
    def __init__(self, config):
        self.front = Camera(config["cameras"]["front"])
        self.rear = Camera(config["cameras"]["rear"])
        self.lidar = Lidar(config["lidar"])

    def start(self):
        for worker in (self.front, self.rear, self.lidar):
            worker.start()

    def close(self):
        for worker in (self.front, self.rear, self.lidar):
            worker.close()
