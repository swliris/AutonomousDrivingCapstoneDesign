"""Sensor-only observer. Its temporary frame starts at the vehicle's first scan."""
import math

import numpy as np

from .config import vehicle_from
from .geometry import external_points, inverse, parking_valid, swept_contact, tire_polygons, transform
from .localization import ScanOdometry
from .markings import RoadMarks, outside_road, road_corridor
from .parking_scene import ParkingScene
from .perception import Vision
from .types import Light, Observation


class RelativeObserver:
    def __init__(self, config, mode):
        self.config = config
        self.vehicle = vehicle_from(config)
        self.vision = Vision(config)
        self.odometry = ScanOdometry(config["safety"]["icp_max_rmse_m"])
        self.road = None if mode == "parking" else RoadMarks(config, self.vehicle)
        self.parking = ParkingScene(config, self.vehicle) if mode == "parking" else None
        self.lidar_stamp = self.front_stamp = self.rear_stamp = -math.inf
        self.localized = False
        self.lanes = {}
        self.last_paths = {}
        self.lane_stamp = -math.inf
        self.current_lane = 2
        self.light = Light.UNKNOWN
        self.view = None
        self.points = np.empty((0, 2))
        self.angles = self.ranges = np.empty(0)
        self.previous_front = self.previous_tires = None
        self.gate = False

    def observe(self, front, rear, lidar, feedback, *, now):
        limits = self.config["safety"]
        new_scan = lidar is not None and lidar.stamp > self.lidar_stamp
        if new_scan:
            if lidar.stamp-self.lidar_stamp > limits["lidar_max_age_s"] or not self.localized:
                self.previous_front = self.previous_tires = None
            self.lidar_stamp = lidar.stamp
            raw = external_points(lidar.data["points"], self.vehicle)
            pose, self.localized = self.odometry.update(raw, lidar.stamp)
            self.points = transform(raw, pose)
            self.angles, self.ranges = lidar.data["angles"], lidar.data["ranges"]
        pose = self.odometry.pose
        pose_stamp = self.odometry.stamp
        def aligned(stamp):
            return self.localized and pose_stamp is not None and abs(stamp-pose_stamp) <= limits["pose_max_age_s"]

        if front and front.stamp > self.front_stamp:
            self.front_stamp = front.stamp
            if self.road and aligned(front.stamp):
                previous = self.last_paths if front.stamp-self.lane_stamp < .7 else {}
                self.lanes, mask, self.view = self.vision.lanes(front.data, pose, previous=previous,
                                                               current_lane=self.current_lane)
                if self.lanes:
                    self.last_paths, self.lane_stamp = self.lanes, front.stamp
                    centers = {k: abs(np.interp(.55, *inverse([(p.x, p.y) for p in path], pose).T))
                               for k, path in self.lanes.items()}
                    nearest = min(centers, key=centers.get)
                    if centers[nearest] < .28*self.road.width:
                        self.current_lane = nearest
                self.road.see_paint(mask, self.lanes or previous, pose)
            else:
                self.lanes = {}
                self.view = self.vision.undistort(front.data, "front")
                if self.parking and aligned(front.stamp):
                    self.parking.see_paint("front", self.vision.mask(self.view), pose, front.stamp)
            if self.road:
                self.light = self.vision.light(self.view)
        if rear and rear.stamp > self.rear_stamp:
            self.rear_stamp = rear.stamp
            if self.parking and aligned(rear.stamp):
                frame = self.vision.undistort(rear.data, "rear")
                self.parking.see_paint("rear", self.vision.mask(frame), pose, rear.stamp)

        obs = Observation(now, self.front_stamp, self.rear_stamp, self.lidar_stamp, pose, self.localized,
                          pose_stamp=pose_stamp if pose_stamp is not None else -math.inf,
                          feedback_stamp=feedback.stamp if feedback and not feedback.status & 2 else -math.inf,
                          speed=self.odometry.speed, obstacles=self.points, scan_angles=self.angles, scan_ranges=self.ranges,
                          light=self.light, light_stamp=self.front_stamp, lanes=self.lanes, scene={})
        if self.road:
            obs.scene["road_polygon"] = road_corridor(self.lanes, pose, self.vehicle, self.road.width)
            obs.stop_zone_distance, obs.in_stop_zone = self.road.stop_zone(pose)
            steer = self.steering(feedback)
            obs.outside = outside_road(self.lanes, pose, self.vehicle, self.road.width, steer)
            if new_scan:
                self.gate = False
                if self.localized:
                    fresh_lanes = self.lanes if aligned(self.front_stamp) else {}
                    self.road.see_posts(raw, fresh_lanes, pose, self.lidar_stamp)
                    v = self.vehicle
                    front_edge = transform([[v.front, -v.width/2], [v.front, v.width/2]], pose)
                    if self.road.gate is not None:
                        self.gate = swept_contact([front_edge], self.previous_front, self.road.gate)
                    self.previous_front = [front_edge]
            obs.gate = self.gate
            if self.gate and self.road.gate is not None:
                obs.scene["timer_line"] = self.road.gate.tolist()
        elif self.localized:
            park = self.parking.update(pose, self.points, now)
            obs.scene["parking"] = park
            steer = self.steering(feedback)
            tires = tire_polygons(pose, self.vehicle, steer)
            obs.parallel_entry = any(swept_contact(tires, self.previous_tires, edge) for edge in park["parallel_entry_lines"])
            if park["out_line"]:
                obs.out_contact = swept_contact(tires, self.previous_tires, park["out_line"])
            if rear and 0 <= now-rear.stamp <= limits["camera_max_age_s"] and aligned(rear.stamp):
                for kind in ("perpendicular", "parallel"):
                    for number, slot in park[kind].items():
                        if (parking_valid(pose, self.vehicle, slot, kind == "perpendicular", steer)
                                and self.vision.parking_alignment(rear.data, "rear", pose, slot)):
                            obs.parked_valid, obs.parked_slot = True, f"{kind}:{number}"
            obs.outside = self.parking.outside(tires)
            self.previous_tires = tires
        return obs

    def steering(self, feedback):
        if feedback is None:
            return 0.
        table = np.asarray(self.config["calibration"]["pot_to_steer"])
        return float(np.interp(feedback.potentiometer, *table.T))
