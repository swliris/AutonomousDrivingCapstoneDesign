"""Offline perception. No camera workers, serial ports or actuator threads."""
from contextlib import contextmanager
import json
import math
from pathlib import Path
import re
import time

import cv2
import numpy as np

from .perception import Vision
from .runtime import Observer, observation_dict, start_pose


@contextmanager
def new_log(output):
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    stream = path.open("x", encoding="utf-8")
    try:
        with stream:
            yield stream
    except BaseException:
        path.unlink(missing_ok=True)
        raise


def wait_for_frame(deadline):
    while True:
        remaining = deadline - time.monotonic()
        delay = max(1, min(30, int(remaining * 1000)))
        if cv2.waitKey(delay) & 255 in (27, ord("q")):
            return False
        if remaining <= .001:
            return True


def scan_view(points, radius):
    # Vehicle frame: forward is up, left is left. The origin is the rear axle.
    image = np.zeros((401, 401, 3), np.uint8)
    pixels = np.rint(200 - points[:, [1, 0]] * (200 / radius)).astype(int)
    visible = pixels[((pixels >= 0) & (pixels <= 400)).all(axis=1)]
    image[visible[:, 1], visible[:, 0]] = (0, 230, 100)
    cv2.arrowedLine(image, (200, 200), (200, 186), (255, 255, 255), 1)
    return image


def replay_raw(recording, config, course, mode, start_id, output, view=False):
    cameras = ("front", "rear") if mode == "parking" else ("front",)
    for name in cameras:
        camera = config["cameras"][name]
        if any(camera.get(key) is None for key in ("intrinsics", "ground_homography")):
            raise ValueError(f"missing {name} camera calibration: supply a calibrated --config")
    observer = Observer(config, course, mode, start_pose(course, mode, start_id))
    first_stamp = recording.events[0][0]
    started = time.monotonic()
    count = 0
    try:
        with new_log(output) as log:
            for stamp, front, rear, lidar in recording.snapshots(config):
                # record() has no Arduino feedback or operator start events.
                obs = observer.observe(front, rear, lidar, None, now=stamp)
                if view:
                    if not wait_for_frame(started + (stamp - first_stamp)):
                        break
                    if observer.view is not None:
                        cv2.imshow("Replay front", observer.view)
                    if rear is not None:
                        cv2.imshow("Replay rear", rear.data)
                    if lidar is not None:
                        cv2.imshow("Replay LiDAR", scan_view(lidar.data["points"], config["lidar"]["max_range_m"]))
                log.write(json.dumps(observation_dict(obs), allow_nan=False) + "\n")
                count += 1
    finally:
        if view:
            cv2.destroyAllWindows()
    return {"observations": count, "output": str(output)}


def replay_images(directory, config, output, view=False, fps=10.):
    """Pixel-space playback for numbered, uncalibrated single-camera sequences."""
    if not math.isfinite(fps) or fps <= 0:
        raise ValueError("preview fps must be positive")
    frames = []
    for path in Path(directory).iterdir():
        if path.suffix.lower() in (".jpg", ".jpeg", ".png"):
            match = re.match(r"^(\d+)(?:_|$)", path.stem)
            if not match:
                raise ValueError(f"image has no numeric frame prefix: {path.name}")
            frames.append((int(match[1]), path))
    frames.sort()
    if not frames or len({index for index, _ in frames}) != len(frames):
        raise ValueError("expected images with unique numeric frame indices")
    vision = Vision(config)
    count = 0
    started = time.monotonic()
    try:
        with new_log(output) as log:
            for index, path in frames:
                image = cv2.imread(str(path), cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError(f"unreadable image: {path}")
                mask = vision.mask(image)
                if view:
                    if not wait_for_frame(started + (index - frames[0][0]) / fps):
                        break
                    cv2.imshow("Camera", image)
                    cv2.imshow("Paint mask", mask)
                row = {"frame": index, "file": path.name, "paint_pixels": int(np.count_nonzero(mask)),
                       "width": image.shape[1], "height": image.shape[0]}
                log.write(json.dumps(row, ensure_ascii=False) + "\n")
                count += 1
    finally:
        if view:
            cv2.destroyAllWindows()
    return {"images": count, "output": str(output)}
