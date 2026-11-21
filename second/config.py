import json
import math
from pathlib import Path
import numpy as np
from .types import Vehicle


ROOT = Path(__file__).resolve().parent.parent


def load_json(path):
    path = Path(path).resolve()
    with path.open(encoding="utf-8") as stream:
        result = json.load(stream)
    return result


def vehicle_from(config):
    return Vehicle(**config["vehicle"])


def validate(config, course, mode):
    errors = []
    if not config.get("calibrated", False):
        errors.append("vehicle.calibrated=false: vehicle example values have not been measured")
    if not course.get("surveyed", False):
        errors.append("course.surveyed=false: map coordinates and actual line edges must be surveyed")
    if config["serial"]["protocol"] != "v2":
        errors.append("live mode requires paired watchdog/feedback v2 firmware (legacy is replay/bench only)")
    if config["serial"]["baud"] != 115200:
        errors.append("v2 firmware baud must be 115200")
    if not config.get("firmware_verified", False):
        errors.append("firmware_verified=false: confirm uploaded firmware, polarity, stop and steering on raised wheels")
    if config["vehicle"]["width"] > .60 or config["vehicle"]["front"]+config["vehicle"]["rear"] > 1.10 or config.get("height_m", 100.) > .75:
        errors.append("vehicle + sensors + jig exceeds 1.10 x .60 x .75 m rule envelope")
    for name in ("front", "rear"):
        camera = config["cameras"][name]
        if not camera.get("intrinsics") or not camera.get("ground_homography"):
            errors.append(f"{name} camera intrinsics and ground homography are required")
        if not camera.get("device"):
            errors.append(f"{name} stable camera device path is missing")
        for field in ("intrinsics", "ground_homography"):
            matrix = np.asarray(camera.get(field), float)
            if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)) or abs(np.linalg.det(matrix)) < 1e-10:
                errors.append(f"{name}.{field} must be a finite invertible 3x3 matrix")
    map_path = Path(course.get("lidar_map", ""))
    if not map_path.is_file():
        errors.append("surveyed static LiDAR point map .npy is missing")
    if mode in ("mission1", "signal_only"):
        light = config["perception"]["signal"]
        if not light.get("model") and not (light.get("roi") and light.get("roi_verified")):
            errors.append("signal detector weights or a track-validated fixed signal ROI is required")
        if not course.get("stop_zone", {}).get("inner_polygon") or not course.get("stop_zone", {}).get("target_pose"):
            errors.append("designated stop-zone inner polygon and target pose are missing")
    if mode == "parking":
        parking = course.get("parking", {})
        if not parking.get("area_polygon") or not parking.get("out_line") or not parking.get("parallel_entry_lines"):
            errors.append("parking area, OUT line and ALL parallel-entry lines are required")
        for kind in ("perpendicular", "parallel"):
            for number in ("1", "2"):
                slot = parking.get(kind, {}).get(number, {})
                if not all(slot.get(key) for key in ("inner_polygon", "outer_polygon", "goal", "approach", "painted_edges")):
                    errors.append(f"missing measured {kind} slot {number}")
        if len(parking.get("starts", {})) != 4 or not all(parking.get("starts", {}).values()):
            errors.append("parking requires all four right-front-tire start poses")
    elif not all(course.get("lane_paths", {}).get(str(lane)) for lane in (1, 2)) or not course.get("road_polygon") or not course.get("lap_gate"):
        errors.append("road perimeter, both reference lanes and lap-timer gate are required")
    try:
        v = vehicle_from(config)
        if min(v.wheelbase, v.track, v.tire_width, v.tire_length) <= 0 or v.front <= v.wheelbase or not 0 < v.max_steer < math.pi/2:
            errors.append("invalid vehicle geometry")
        for name in ("speed_to_pwm", "steer_to_command", "pot_to_steer"):
            table = np.asarray(config["calibration"][name], float)
            if table.ndim != 2 or table.shape[1] != 2 or not np.all(np.isfinite(table)) or not np.all(np.diff(table[:, 0]) > 0):
                errors.append(f"{name} must contain finite strictly increasing measured input values")
        table = np.asarray(config["calibration"]["steer_to_command"], float)
        if table[0, 0] > -v.max_steer or table[-1, 0] < v.max_steer or np.any(np.abs(table[:, 1]) > 120):
            errors.append("steering lookup must cover physical steering range within firmware +/-120")
    except (ValueError, TypeError, KeyError, IndexError):
        errors.append("malformed vehicle/calibration settings")
    return errors


class MotorMapper:
    def __init__(self, config):
        self.speed = np.asarray(config["calibration"]["speed_to_pwm"], float)
        self.steer = np.asarray(config["calibration"]["steer_to_command"], float)

    def convert(self, command):
        if not math.isfinite(command.speed) or not math.isfinite(command.steer):
            return 0, 0
        pwm = 0 if command.speed == 0 else int(round(np.interp(command.speed, self.speed[:, 0], self.speed[:, 1])))
        steer = int(round(np.interp(command.steer, self.steer[:, 0], self.steer[:, 1])))
        return int(np.clip(pwm, -255, 255)), int(np.clip(steer, -120, 120))
