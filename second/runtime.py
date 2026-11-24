import json
import logging
import math
from pathlib import Path
import threading
import time
import cv2
import numpy as np
from .config import MotorMapper, vehicle_from
from .geometry import contained, footprint, inside, inverse, external_points, parking_valid, swept_contact, tire_polygons, transform
from .localization import Localizer
from .mission import Mission
from .perception import Vision
from .protocol import SerialDriver
from .sensors import SensorHub
from .types import Command, Light, Observation, Pose, wrap

LOG = logging.getLogger(__name__)


class Actuator:
    """Only this thread writes motor commands. Inference/planning never blocks it."""
    def __init__(self, config):
        self.driver = SerialDriver(config["serial"])
        self.mapper = MotorMapper(config)
        self.config = config
        self.command = Command()
        self.stamp = -math.inf
        self.enabled = False
        self.closed = threading.Event()
        self.lock = threading.Lock()
        self.error = None
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def submit(self, command, observation_stamp, enabled):
        with self.lock:
            self.command, self.stamp, self.enabled = command, observation_stamp, enabled

    def run(self):
        try:
            while not self.closed.wait(.04):
                with self.lock:
                    command, stamp, enabled = self.command, self.stamp, self.enabled
                alive = enabled and 0 <= time.monotonic()-stamp <= self.config["safety"]["command_max_age_s"]
                pwm, steer = self.mapper.convert(command) if alive else (0, 0)
                self.driver.send(pwm, steer, alive)
                self.driver.poll()
        except Exception as exc:
            self.error = str(exc)
            LOG.exception("actuator failed")
        finally:
            self.driver.close()

    def close(self):
        self.closed.set()
        self.thread.join(timeout=2.)


def start_pose(course, mode, start_id):
    starts = course["parking"]["starts"] if mode == "parking" else course["road_starts"]
    coords = starts.get(start_id)
    if coords is None or len(coords) != 3:
        raise ValueError(f"missing surveyed start pose for {mode} start {start_id!r}")
    return Pose(*coords)


class Observer:
    def __init__(self, config, course, mode, initial):
        self.config, self.course, self.mode = config, course, mode
        self.vehicle = vehicle_from(config)
        self.vision = Vision(config)
        self.localizer = Localizer(np.load(course["lidar_map"], allow_pickle=False), initial,
                                   config["safety"]["icp_max_rmse_m"])
        self.lidar_stamp = -math.inf
        self.front_stamp = -math.inf
        self.localized = False
        self.light = Light.UNKNOWN
        self.lanes = {}
        self.view = None
        self.previous_tires = None
        self.previous_front = None
        self.points = np.empty((0, 2))
        self.angles = self.ranges = np.empty(0)

    def observe(self, front, rear, lidar, feedback, *, now=None):
        if now is None:
            now = time.monotonic()
        v = self.vehicle
        if lidar and lidar.stamp > self.lidar_stamp:
            self.lidar_stamp = lidar.stamp
            raw = external_points(lidar.data["points"], v)
            self.localizer.update(raw, lidar.stamp)
            self.localized = self.localizer.stamp == lidar.stamp
            self.points = transform(raw, self.localizer.pose)
            self.angles, self.ranges = lidar.data["angles"], lidar.data["ranges"]
        pose = self.localizer.pose
        if front and front.stamp > self.front_stamp:
            self.front_stamp = front.stamp
            if self.mode == "parking":
                self.view = self.vision.undistort(front.data, "front")
            else:
                self.lanes, mask, self.view = self.vision.lanes(front.data, pose, self.course)
            self.light = self.vision.light(self.view)
        steer = 0.
        if feedback:
            table = np.asarray(self.config["calibration"]["pot_to_steer"])
            steer = float(np.interp(feedback.potentiometer, table[:, 0], table[:, 1]))
        tires = tire_polygons(pose, v, steer)
        front_edge = transform([[v.front, -v.width/2], [v.front, v.width/2]], pose)
        gate = False
        if self.course.get("lap_gate"):
            gate = swept_contact([front_edge], [self.previous_front] if self.previous_front is not None else None, self.course["lap_gate"])
        self.previous_front = front_edge
        obs = Observation(now, front.stamp if front else -math.inf, rear.stamp if rear else -math.inf,
                          self.lidar_stamp, pose, self.localized, speed=self.localizer.speed,
                          light=self.light, light_stamp=self.front_stamp, lanes=self.lanes,
                          obstacles=self.points, scan_angles=self.angles, scan_ranges=self.ranges,
                          gate=gate, pose_stamp=self.localizer.stamp if self.localizer.stamp is not None else -math.inf,
                          feedback_stamp=feedback.stamp if feedback and not feedback.status & 2 else -math.inf)
        if self.mode == "parking":
            park = self.course["parking"]
            obs.parallel_entry = any(swept_contact(tires, self.previous_tires, line) for line in park["parallel_entry_lines"])
            obs.out_contact = swept_contact(tires, self.previous_tires, park["out_line"])
            obs.outside = not any(np.any(inside(poly, park["area_polygon"])) for poly in tires)
            for kind in ("perpendicular", "parallel"):
                for name, slot in park[kind].items():
                    if (parking_valid(pose, v, slot, kind == "perpendicular", steer)
                            and rear and now-rear.stamp <= self.config["safety"]["camera_max_age_s"]
                            and self.vision.parking_alignment(rear.data, "rear", pose, slot)):
                        obs.parked_valid, obs.parked_slot = True, f"{kind}:{name}"
        else:
            zone = self.course.get("stop_zone")
            if zone:
                obs.in_stop_zone = contained(footprint(pose, v), zone["inner_polygon"])
                target = Pose(*zone["target_pose"])
                local = inverse([[target.x, target.y]], pose)[0]
                if -.08 <= local[0] <= 8. and abs(local[1]) < 1.2 and abs(wrap(pose.yaw-target.yaw)) < .8:
                    obs.stop_zone_distance = max(0., float(local[0]))
            region = self.course["road_polygon"]
            obs.outside = not all(contained(poly, region) for poly in tires)
        self.previous_tires = tires
        return obs


class Journal:
    def __init__(self, directory):
        self.path = Path(directory)
        self.path.mkdir(parents=True, exist_ok=True)
        self.file = (self.path/"observations.jsonl").open("w", encoding="utf-8")

    def write(self, obs, command, mission):
        row = observation_dict(obs)
        row["command"] = {"pwm": command.pwm, "steer": command.steer, "reason": command.reason, "state": command.state}
        row["events"] = mission.events
        self.file.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+"\n")
        self.file.flush()

    def close(self):
        self.file.close()


def observation_dict(obs):
    def finite(v):
        return v if v is None or math.isfinite(v) else -1e30
    return {"stamp": obs.stamp, "front_stamp": finite(obs.front_stamp), "rear_stamp": finite(obs.rear_stamp),
            "lidar_stamp": finite(obs.lidar_stamp), "pose": obs.pose.vector().tolist(), "localized": obs.localized,
            "speed": obs.speed, "light": obs.light.value, "light_stamp": finite(obs.light_stamp),
            "lanes": {str(k): [[p.x, p.y, p.yaw, p.direction] for p in path] for k, path in obs.lanes.items()},
            "obstacles": obs.obstacles.tolist(), "scan_angles": obs.scan_angles.tolist(), "scan_ranges": obs.scan_ranges.tolist(),
            **{k: getattr(obs, k) for k in ("stop_zone_distance", "in_stop_zone", "gate", "parallel_entry", "out_contact", "parked_valid", "parked_slot", "parking_line_contact", "outside", "collision")},
            "scene": obs.scene, "pose_stamp": finite(obs.pose_stamp), "feedback_stamp": finite(obs.feedback_stamp)}


def observation_from(row):
    from .types import Waypoint
    r = {k: v for k, v in row.items() if k in Observation.__dataclass_fields__}
    r["pose"] = Pose(*r["pose"])
    r["light"] = Light(r.get("light", "unknown"))
    r["lanes"] = {int(k): [Waypoint(*p) for p in path] for k, path in r.get("lanes", {}).items()}
    for key in ("obstacles", "scan_angles", "scan_ranges"):
        if key in r:
            r[key] = np.asarray(r[key], float)
    return Observation(**r)


def make_observer(config, course, mode, start_id):
    if course is None:
        from .relative import RelativeObserver
        return RelativeObserver(config, mode)
    return Observer(config, course, mode, start_pose(course, mode, start_id))


def live(config, course, mode, start_id, perpendicular, parallel, log_dir):
    mission = Mission(config, course, mode, vehicle_from(config), perpendicular, parallel)
    observer = make_observer(config, course, mode, start_id)
    hub = SensorHub(config)
    journal = Journal(log_dir)
    actuator = None
    try:
        hub.start()
        actuator = Actuator(config)
        print("G: start AFTER the start sound ends. Space/Q: stop. No motors before G.", flush=True)
        while True:
            if actuator.error:
                raise RuntimeError(actuator.error)
            obs = observer.observe(hub.front.latest(), hub.rear.latest(), hub.lidar.latest(), actuator.driver.feedback, now=time.monotonic())
            command = mission.update(obs)
            actuator.submit(command, obs.stamp, mission.state not in ("idle", "done", "failed"))
            if observer.view is not None:
                view = observer.view.copy()
                cv2.putText(view, f"{mode} | {mission.state} | {command.reason}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 200, 255), 1)
                cv2.imshow("Second front - G start / SPACE stop", view)
            rear = hub.rear.latest()
            if rear:
                cv2.imshow("Second rear", rear.data)
            key = cv2.waitKey(1) & 255
            if key in (ord("q"), 32, 27):
                mission.stop(time.monotonic())
                actuator.submit(Command(), time.monotonic(), False)
                break
            if key == ord("g"):
                mission.start(time.monotonic())
            journal.write(obs, command, mission)
            if mission.state in ("done", "failed"):
                break
            time.sleep(.01)
    finally:
        if actuator:
            actuator.close()
        hub.close()
        journal.close()
        cv2.destroyAllWindows()
    return mission


def record(config, directory, duration):
    """Read-only sensor collection: never opens the Arduino port."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    for name in ("front", "rear", "lidar"):
        (directory/name).mkdir()
    (directory/"sensor_config.json").write_text(json.dumps(config, ensure_ascii=False, indent=2), encoding="utf-8")
    hub = SensorHub(config)
    seen = {"front": -1., "rear": -1., "lidar": -1.}
    count = {"front": 0, "rear": 0, "lidar": 0}
    hub.start()
    started = time.monotonic()
    try:
        with (directory/"frames.jsonl").open("w", encoding="utf-8") as log:
            while time.monotonic()-started < duration:
                for name in seen:
                    worker = getattr(hub, name)
                    if worker.error:
                        raise RuntimeError(f"{name}: {worker.error}")
                    sample = worker.latest()
                    if sample is None or sample.stamp <= seen[name]:
                        continue
                    seen[name] = sample.stamp
                    stem = f"{count[name]:07d}"
                    path = directory/name/(stem+(".npz" if name == "lidar" else ".jpg"))
                    if name == "lidar":
                        np.savez_compressed(path, **sample.data)
                    elif not cv2.imwrite(str(path), sample.data, [cv2.IMWRITE_JPEG_QUALITY, 96]):
                        raise IOError(f"cannot save {path}")
                    log.write(json.dumps({"sensor": name, "stamp": sample.stamp, "path": str(path.relative_to(directory))})+"\n")
                    count[name] += 1
                time.sleep(.005)
    finally:
        hub.close()
    if not all(count.values()):
        raise RuntimeError(f"incomplete sensor recording: {count}")
    return count
