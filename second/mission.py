"""Competition policy. Judge decisions remain external; these are local guards."""
import math
from dataclasses import dataclass
import numpy as np
from .config import drive_control_errors
from .geometry import inverse, path_clear
from .planning import Tracker, hybrid_astar, lane_change, observed_free, turning_paths
from .protocol import MAX_DRIVE_PWM
from .types import Command, Light, Pose, wrap


@dataclass
class ParkingHold:
    duration: float = 5.2  # margin over the required five seconds
    since: float | None = None
    anchor: Pose | None = None

    def update(self, now, pose, valid, speed):
        moved = self.anchor is not None and (math.hypot(pose.x-self.anchor.x, pose.y-self.anchor.y) > .015
                                             or abs(wrap(pose.yaw-self.anchor.yaw)) > math.radians(1.5))
        if not valid or abs(speed) > .02 or moved:
            self.since = self.anchor = None
        if valid and abs(speed) <= .02 and self.since is None:
            self.since, self.anchor = now, pose
        return self.since is not None and now-self.since >= self.duration


class Mission:
    MODES = ("mission1", "signal_only", "parking")

    def __init__(self, config, course, mode, vehicle, perpendicular="1", parallel="1"):
        if mode not in self.MODES:
            raise ValueError(mode)
        errors = drive_control_errors(config["control"])
        if errors:
            raise ValueError("\n".join(errors))
        self.config, self.course, self.mode, self.vehicle = config, course, mode, vehicle
        self.selected = {"perpendicular": perpendicular, "parallel": parallel}
        self.state = "idle"
        self.started = None
        self.first_motion = None
        self.events = []
        self.tracker = Tracker(vehicle)
        self.lane = 2
        self.destination_lane = 2
        self.changing = False
        self.travel = 0.
        self.previous_pose = None
        self.previous_stamp = None
        self.gate_high = False
        self.first_gate = None
        self.timer_center = None
        self.last_gate_travel = -math.inf
        self.laps = 0
        self.green_since = None
        self.green_last_stamp = None
        self.red_latched = False
        self.signal_done = False
        self.signal_left_start = mode == "signal_only"
        self.stopped_since = None
        self.reverse_seen = False
        self.hold = ParkingHold()
        self.parking_results = {"perpendicular": False, "parallel": False}
        self.parking_attempts = set()
        self.searching_aisle = False
        self.entry_high = False
        self.next_plan_at = 0.
        self.previous_command = Command()
        self.blocked_since = None
        self.scores = []

    def event(self, now, code, detail):
        self.events.append({"time": now, "code": code, "detail": detail})

    def start(self, now):
        if self.state != "idle":
            return
        self.started = now
        self.state = "perpendicular_approach" if self.mode == "parking" else "initial_green" if self.mode in ("mission1", "signal_only") else "lane"
        self.event(now, "start_key", self.mode)

    def stop(self, now, reason="operator stop"):
        self.state = "done"
        self.event(now, "stop", reason)
        self.previous_command = Command(reason=reason, state=self.state)
        return self.previous_command

    def finish_command(self, pwm=0, steer=0., reason="hold", path=None):
        self.previous_command = Command(pwm, steer, reason, self.state, path or [])
        return self.previous_command

    def fail(self, now, reason):
        self.state = "failed"
        self.event(now, "failure", reason)
        return self.finish_command(reason=reason)

    def green_confirmed(self, obs):
        fresh = 0 <= obs.stamp-obs.light_stamp <= self.config["safety"]["camera_max_age_s"]
        if not fresh or obs.light != Light.GREEN:
            self.green_since = self.green_last_stamp = None
            return False
        # Reusing one camera frame can never accumulate a green confirmation.
        if self.green_last_stamp is None or obs.light_stamp > self.green_last_stamp:
            if self.green_last_stamp is not None and obs.light_stamp-self.green_last_stamp > .25:
                self.green_since = None
            if self.green_since is None:
                self.green_since = obs.light_stamp
            self.green_last_stamp = obs.light_stamp
        return self.green_last_stamp-self.green_since >= self.config["control"]["green_confirm_s"]

    def update(self, obs):
        now = obs.stamp
        if self.state in ("idle", "done", "failed"):
            return self.finish_command(reason=self.state)
        limits = {"mission1": 240., "signal_only": 60., "parking": 240.}
        if now-self.started > limits[self.mode]:
            return self.fail(now, "competition time limit")
        if obs.collision:
            return self.fail(now, "collision/contact suspected")
        if obs.outside:
            return self.fail(now, "outside permitted area")
        safety = self.config["safety"]
        stamps = [(obs.front_stamp, safety["camera_max_age_s"]), (obs.lidar_stamp, safety["lidar_max_age_s"])]
        if self.mode == "parking":
            stamps.append((obs.rear_stamp, safety["camera_max_age_s"]))
        if obs.pose_stamp is not None:
            stamps.append((obs.pose_stamp, safety["pose_max_age_s"]))
        if self.config["serial"]["protocol"] == "v2":
            stamps.append((obs.feedback_stamp if obs.feedback_stamp is not None else -math.inf, safety["feedback_max_age_s"]))
        healthy = obs.localized and all(0 <= now-stamp <= age for stamp, age in stamps)
        if not healthy:
            self.green_since = self.green_last_stamp = None
            self.hold.update(now, obs.pose, False, obs.speed)
            return self.finish_command(reason="stale sensor, pose, or motor feedback")
        if self.previous_stamp is not None and now <= self.previous_stamp:
            return self.finish_command(reason="non-monotonic observation")
        if self.previous_pose is not None:
            self.travel += math.hypot(obs.pose.x-self.previous_pose.x, obs.pose.y-self.previous_pose.y)
        self.previous_pose, self.previous_stamp = obs.pose, now
        if abs(obs.speed) > .025 and self.first_motion is None:
            self.first_motion = now
        if self.first_motion is None and now-self.started >= 15.:
            return self.fail(now, "not started within fifteen seconds")
        if self.mode == "parking":
            return self.parking(obs)
        result = self.road(obs)
        intentional_wait = self.state in ("initial_green", "wait_green")
        if not intentional_wait and abs(obs.speed) < .02:
            if self.blocked_since is None:
                self.blocked_since = now
            elif now-self.blocked_since >= 5.:
                return self.fail(now, "unable to advance for five seconds")
        else:
            self.blocked_since = None
        return result

    def road(self, obs):
        now = obs.stamp
        green = self.green_confirmed(obs)
        if self.state == "initial_green":
            if not green:
                if now-self.started >= 15.:
                    return self.fail(now, "not started within fifteen seconds")
                return self.finish_command(reason="waiting for initial green after start key")
            self.state = "lane"
            self.green_since = self.green_last_stamp = None
        if not obs.in_stop_zone and self.travel > 2.:
            self.signal_left_start = True
        # Gate uses foremost sensor/jig edge (runtime computes gate contact).
        if obs.scene is not None:
            # The return pass follows the completed stop/red/green sequence.
            line = obs.scene.get("timer_line")
            center = np.mean(line, axis=0) if line is not None else None
            if self.first_gate is None:
                gate_eligible = center is not None and not self.signal_done
            else:
                gate_eligible = (self.signal_done and center is not None and self.timer_center is not None
                                 and np.linalg.norm(center-self.timer_center) < 1.2)
        else:
            gate_eligible = self.course is not None and self.travel-self.last_gate_travel >= self.course["minimum_lap_travel_m"]
        if obs.gate and not self.gate_high and gate_eligible:
            if self.first_gate is None:
                self.first_gate = now
                if obs.scene is not None:
                    self.timer_center = center
            else:
                self.laps += 1
            self.last_gate_travel = self.travel
        self.gate_high = obs.gate
        laps_required = 1
        if self.laps >= laps_required:
            if self.mode == "mission1" and not self.signal_done:
                self.event(now, "signal_incomplete", "lap ended without red-stop-green")
            return self.stop(now, "required lap count completed")
        signal_mode = self.mode in ("mission1", "signal_only") and self.signal_left_start and not self.signal_done
        d = obs.stop_zone_distance
        if signal_mode and obs.light == Light.RED and now-obs.light_stamp <= self.config["safety"]["camera_max_age_s"]:
            self.red_latched = True
        if self.state == "wait_green":
            if not obs.in_stop_zone or abs(obs.speed) > .02:
                self.stopped_since = None
            elif self.stopped_since is None:
                self.stopped_since = now
            if self.red_latched and self.stopped_since is not None and now-self.stopped_since >= .25 and green:
                self.signal_done = True
                self.red_latched = False
                self.state = "lane"
                self.changing = False
                self.tracker.set_path([])
                self.event(now, "signal_success", "red stop in designated zone then confirmed green")
                if self.mode == "signal_only":
                    self.state = "signal_depart"
            else:
                return self.finish_command(reason="red/unknown signal latched; awaiting fresh green")
        if self.state == "signal_depart" and not obs.in_stop_zone:
            return self.stop(now, "signal-only departure completed")
        control = self.config["control"]
        stop_distance = obs.speed**2/(2*control["deceleration_mps2"])+abs(obs.speed)*control["latency_budget_s"]+.04
        if signal_mode and d is not None and d <= max(.06, stop_distance) and not self.signal_done:
            self.state = "wait_green"
            self.stopped_since = now if obs.in_stop_zone and abs(obs.speed) < .02 else None
            # A first green seen while moving cannot cancel the required red stop.
            self.green_since = self.green_last_stamp = None
            return self.finish_command(reason="stopping in designated signal zone")
        geometry = obs.scene if obs.scene is not None else self.course
        allowed = geometry.get("road_polygon") if geometry else None
        if allowed is None:
            return self.finish_command(reason="road boundaries not observed")
        if obs.scene is not None and signal_mode and self.red_latched and d is None:
            return self.finish_command(reason="red light; crosswalk position unknown")
        # Limit approach output before the LiDAR-based stop point.
        approaching_stop = signal_mode and d is not None and d <= .30
        pwm_limit = control["curve_pwm"] if approaching_stop else MAX_DRIVE_PWM
        if self.changing:
            # A stopped/restarted or obstructed manoeuvre is re-evaluated, never a timed steering script.
            remaining = self.tracker.path[self.tracker.index:]
            if not path_clear(remaining, obs.obstacles, self.vehicle, allowed) or not self.free(remaining, obs):
                return self.finish_command(reason="lane change blocked or unobserved")
            steer, direction, done = self.tracker.update(obs.pose, obs.speed, now)
            if done:
                self.lane = self.destination_lane
                self.changing = False
            else:
                return self.finish_command(min(pwm_limit, control["lane_change_pwm"]), steer, "tracking lane change", remaining)
        path = self.near_lane(obs, self.lane)
        if not path:
            return self.finish_command(reason="current lane not observed")
        if obs.scene is not None:
            # Leave room inside the observed corridor for the nose at the
            # last commanded waypoint. Never extend paint into unseen space.
            forward = inverse([(p.x, p.y) for p in path], obs.pose)[:, 0]
            end = min(forward[-1], max(inverse(allowed, obs.pose)[:, 0]))-self.vehicle.front-self.vehicle.margin-.03
            path = [p for p, x in zip(path, forward) if x <= end]
            if not path or math.hypot(path[-1].x-obs.pose.x, path[-1].y-obs.pose.y) < self.tracker.lookahead:
                return self.finish_command(reason="insufficient visible lane ahead")
        if not path_clear(path, obs.obstacles, self.vehicle, allowed) or not self.free(path, obs):
            if self.mode != "mission1":
                return self.finish_command(reason="outer lane blocked; lane change disallowed in timed race")
            other = 1 if self.lane == 2 else 2
            target = self.near_lane(obs, other)
            if not target:
                return self.finish_command(reason="adjacent lane not observed")
            for length in (2.0, 2.5, 3.0, 3.5):
                candidate = lane_change(obs.pose, target, self.vehicle, length)
                if candidate and path_clear(candidate, obs.obstacles, self.vehicle, allowed) and self.free(candidate, obs):
                    self.destination_lane, self.changing = other, True
                    self.tracker.set_path(candidate)
                    self.event(now, "lane_change", f"{self.lane}->{other}")
                    return self.finish_command(reason="lane change planned; start next cycle", path=candidate)
            return self.finish_command(reason="no feasible observed lane change")
        self.tracker.set_path(path)
        steer, _, _ = self.tracker.update(obs.pose, obs.speed, now)
        is_curve = abs(steer) >= math.radians(control["curve_steer_deg"])
        pwm = control["curve_pwm"] if is_curve else control["straight_pwm"]
        return self.finish_command(min(pwm, pwm_limit), steer, "lane following", path)

    def near_lane(self, obs, lane_id):
        # A visible section across the bend may start beyond the tracker. Do
        # not steer across that gap or fill it by clamping a lane-change path.
        path = obs.lanes.get(lane_id, [])
        if path:
            start = path[0]
            forward = inverse([(start.x, start.y)], obs.pose)[0, 0]
            if forward <= self.tracker.lookahead:
                return path
        return []

    def free(self, path, obs):
        return observed_free(path, obs.pose, obs.scan_angles, obs.scan_ranges,
                             Pose(*self.config["lidar"]["pose"]), self.vehicle)

    def parking(self, obs):
        now = obs.stamp
        # ANY parallel entry line starts the parallel stage, even the wrong bay.
        rising_entry = obs.parallel_entry and not self.entry_high
        self.entry_high = obs.parallel_entry
        if rising_entry and self.state.startswith("perpendicular"):
            self.event(now, "perpendicular_incomplete", "parallel entry touched before perpendicular completion")
            self.state = "parallel_approach"
            self.tracker.set_path([])
            self.hold = ParkingHold()
            self.reverse_seen = False
        if obs.out_contact:
            if not all(self.parking_results.values()):
                self.event(now, "parking_incomplete", "OUT touched with incomplete parking stage(s)")
            return self.stop(now, "OUT tire contact")
        geometry = obs.scene if obs.scene is not None else self.course
        park = geometry.get("parking") if geometry else None
        if park is None:
            return self.finish_command(reason="parking markings unavailable")
        if self.state == "parking_out":
            if not park.get("out_goal"):
                return self.search_aisle(obs, park)
            self.finish_search()
            goal = Pose(*park["out_goal"])
            return self.parking_motion(obs, goal, False, "parking_out", stop_on_goal=True)
        kind = "perpendicular" if self.state.startswith("perpendicular") else "parallel"
        slot_id = self.selected[kind]
        slot = park[kind].get(slot_id)
        if slot is None:
            return self.search_aisle(obs, park)
        self.finish_search()
        goal = Pose(*slot["goal"])
        self.parking_attempts.add(kind)
        if self.state.endswith("_hold"):
            # Pose-only, intended-target, reverse-entry, zero-motion success conditions.
            valid = obs.parked_valid and obs.parked_slot == f"{kind}:{slot_id}" and self.reverse_seen
            if self.hold.update(now, obs.pose, valid, obs.speed):
                self.parking_results[kind] = True
                if obs.parking_line_contact:
                    self.event(now, "parking_line_penalty", f"{kind}:10")
                self.event(now, "parking_success", kind)
                self.state = "parallel_approach" if kind == "perpendicular" else "parking_out"
                self.tracker.set_path([])
                self.hold = ParkingHold()
                self.reverse_seen = False
            elif not obs.parked_valid:
                self.state = kind+"_reverse"
                self.tracker.set_path([])
            return self.finish_command(reason="valid parking stationary hold")
        if self.state.endswith("_approach"):
            target = Pose(*slot["approach"])
            return self.parking_motion(obs, target, False, kind+"_reverse")
        if obs.speed < -.025 and math.hypot(obs.pose.x-goal.x, obs.pose.y-goal.y) < 1.5:
            self.reverse_seen = True
        if obs.parked_valid and obs.parked_slot == f"{kind}:{slot_id}" and self.reverse_seen:
            self.state = kind+"_hold"
            self.hold = ParkingHold()
            return self.finish_command(reason="target slot reached; start stationary verification")
        return self.parking_motion(obs, goal, True, kind+"_hold")

    def finish_search(self):
        if self.searching_aisle:
            self.tracker.set_path([])
            self.next_plan_at = 0.
            self.searching_aisle = False

    def search_aisle(self, obs, park):
        self.searching_aisle = True
        target = park.get("search")
        if target is None:
            return self.finish_command(reason="waiting for visible parking aisle")
        # Follow the entrance direction while looking for the selected gap or OUT.
        # After leaving a bay the car can be sideways; use the reversible planner.
        return self.parking_motion(obs, Pose(*target), False, self.state)

    def parking_motion(self, obs, goal, reverse, next_state, stop_on_goal=False):
        now = obs.stamp
        geometry = obs.scene if obs.scene is not None else self.course
        park = geometry["parking"]
        allowed = park.get("perpendicular_area", park["area_polygon"]) if self.state.startswith("perpendicular") else park["area_polygon"]
        if not self.tracker.path:
            if now < self.next_plan_at:
                return self.finish_command(reason="parking replan cooldown")
            self.next_plan_at = now+1.
            path = []
            if obs.scene is not None:
                for candidate in turning_paths(obs.pose, goal, self.vehicle, -1 if reverse else 1):
                    if path_clear(candidate, obs.obstacles, self.vehicle, allowed):
                        path = candidate
                        break
            if not path:
                path = hybrid_astar(obs.pose, goal, self.vehicle, obs.obstacles, allowed, final_reverse=reverse,
                                    timeout=self.config["control"]["planner_timeout_s"])
            if not path:
                return self.finish_command(reason="no collision-free parking path")
            self.tracker.set_path(path)
            return self.finish_command(reason="parking plan ready; remain stopped this cycle", path=path)
        remaining = self.tracker.path[self.tracker.index:]
        if not path_clear(remaining, obs.obstacles, self.vehicle, allowed):
            self.tracker.set_path([])
            return self.finish_command(reason="parking obstacle changed; replan")
        # Check the locally commanded segment, not free space hidden beyond a bend.
        preview = remaining[:20]
        if not self.free(preview, obs):
            return self.finish_command(reason="parking swept area not observed free")
        steer, direction, done = self.tracker.update(obs.pose, obs.speed, now)
        if done:
            self.tracker.set_path([])
            if stop_on_goal:
                return self.finish_command(reason="OUT goal reached; waiting for measured tire-line contact")
            if reverse and not obs.parked_valid:
                return self.finish_command(reason="planner goal reached but actual slot containment unverified")
            self.state = next_state
            return self.finish_command(reason="parking segment complete")
        return self.finish_command(direction*self.config["control"]["parking_pwm"], steer, "parking path tracking", remaining)
