import json
import math
from pathlib import Path
import numpy as np
from .protocol import MAX_DRIVE_PWM
from .types import Vehicle


ROOT = Path(__file__).resolve().parent.parent
DRIVE_PWM_KEYS = ("straight_pwm", "curve_pwm", "lane_change_pwm", "parking_pwm")


def load_json(path):
    path = Path(path).resolve()
    with path.open(encoding="utf-8") as stream:
        result = json.load(stream)
    return result


def vehicle_from(config):
    return Vehicle(**config["vehicle"])


def drive_control_errors(control):
    errors = []
    for key in DRIVE_PWM_KEYS:
        value = control.get(key)
        if type(value) is not int or not 0 <= value <= MAX_DRIVE_PWM:
            errors.append(f"control.{key} must be an integer from 0 to {MAX_DRIVE_PWM}")
    threshold = control.get("curve_steer_deg")
    if type(threshold) not in (float, int) or not 0 < threshold < 90:
        errors.append("control.curve_steer_deg must be an angle between 0 and 90 degrees")
    return errors


def validate(config, course, mode):
    errors = drive_control_errors(config["control"])
    if not config.get("calibrated", False):
        errors.append("vehicle.calibrated=false: vehicle example values have not been measured")
    if course is not None and not course.get("surveyed", False):
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
    if course is not None and not Path(course.get("lidar_map", "")).is_file():
        errors.append("surveyed static LiDAR point map .npy is missing")
    if mode in ("mission1", "signal_only"):
        light = config["perception"]["signal"]
        if not light.get("model") and not (light.get("roi") and light.get("roi_verified")):
            errors.append("signal detector weights or a track-validated fixed signal ROI is required")
        if course is not None and not all(course.get("stop_zone", {}).get(k) for k in ("inner_polygon", "target_pose")):
            errors.append("designated stop-zone inner polygon and target pose are missing")
    if course is not None and mode == "parking":
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
    elif course is not None and (not all(course.get("lane_paths", {}).get(str(lane)) for lane in (1, 2))
                                or not course.get("road_polygon") or not course.get("lap_gate")):
        errors.append("road perimeter, both reference lanes and lap-timer gate are required")
    try:
        v = vehicle_from(config)
        if min(v.wheelbase, v.track, v.tire_width, v.tire_length) <= 0 or v.front <= v.wheelbase or not 0 < v.max_steer < math.pi/2:
            errors.append("invalid vehicle geometry")
        for name in ("steer_to_command", "pot_to_steer"):
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
        self.steer = np.asarray(config["calibration"]["steer_to_command"], float)

    def convert(self, command):
        if not math.isfinite(command.pwm) or not math.isfinite(command.steer):
            return 0, 0
        steer = int(round(np.interp(command.steer, self.steer[:, 0], self.steer[:, 1])))
        return int(np.clip(command.pwm, -MAX_DRIVE_PWM, MAX_DRIVE_PWM)), int(np.clip(steer, -120, 120))
