"""Synthetic policy replay and geometric parking plan; never real sensor data."""
import copy
import csv
import json
from pathlib import Path
import numpy as np
from .config import ROOT, load_json, vehicle_from
from .geometry import path_clear
from .mission import Mission, ParkingHold
from .planning import hybrid_astar, with_headings
from .runtime import observation_dict
from .types import Light, Observation, Pose


def fixtures():
    config = copy.deepcopy(load_json(ROOT/"configs/vehicle.example.json"))
    config["vehicle"].update(wheelbase=.30, width=.30, front=.42, rear=.12, track=.24, tire_length=.06, tire_width=.04, max_steer=.60, margin=.025)
    config["control"]["planner_timeout_s"] = 2.
    area = [[-5., -5.], [10., -5.], [10., 5.], [-5., 5.]]
    slot = {"inner_polygon": [[-1., -.6], [1., -.6], [1., .6], [-1., .6]], "outer_polygon": [[-1.05, -.65], [1.05, -.65], [1.05, .65], [-1.05, .65]], "goal": [0., 0., 0.], "approach": [1.5, 0., 0.], "painted_edges": [[[-1, -.6], [1, -.6]], [[-1, .6], [1, .6]]]}
    course = {"surveyed": False, "minimum_lap_travel_m": 10., "road_polygon": area,
              "lane_paths": {"1": [[0., .85], [5., .85]], "2": [[0., 0.], [5., 0.]]},
              "parking": {"area_polygon": area, "out_goal": [3., 0., 0.], "out_line": [[3., -1.], [3., 1.]],
                          "perpendicular": {"1": copy.deepcopy(slot), "2": copy.deepcopy(slot)},
                          "parallel": {"1": copy.deepcopy(slot), "2": copy.deepcopy(slot)}}}
    return config, course


def observation(t, **values):
    x = np.linspace(.15, 4., 70)
    args = dict(stamp=t, front_stamp=t, rear_stamp=t, lidar_stamp=t, feedback_stamp=t, pose_stamp=t,
                pose=Pose(), localized=True, light_stamp=t,
                lanes={2: with_headings(np.column_stack([x, x*0.])), 1: with_headings(np.column_stack([x, x*0.+.85]))},
                scan_angles=np.linspace(-np.pi, np.pi, 1440, endpoint=False), scan_ranges=np.full(1440, 10.))
    args.update(values)
    return Observation(**args)


def run(output):
    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    config, course = fixtures()
    vehicle = vehicle_from(config)
    mission = Mission(config, course, "mission1", vehicle)
    mission.start(0.)
    rows = []
    for t in np.arange(0., .8, .1):
        obs = observation(float(t), light=Light.GREEN, speed=.1)
        cmd = mission.update(obs)
        row = observation_dict(obs); row["start"] = bool(t == 0.); rows.append(row)
    mission.signal_left_start = True
    mission.update(observation(1., light=Light.RED, stop_zone_distance=.01, in_stop_zone=True))
    for t in np.arange(1.1, 2.1, .1):
        mission.update(observation(float(t), light=Light.UNKNOWN, in_stop_zone=True))
    held_on_dropout = mission.state == "wait_green"
    for t in np.arange(2.1, 2.7, .1):
        mission.update(observation(float(t), light=Light.GREEN, in_stop_zone=True))
    (output/"synthetic_observations.jsonl").write_text("".join(json.dumps(r)+"\n" for r in rows))
    path = hybrid_astar(Pose(1.2, 0., 0.), Pose(0., 0., 0.), vehicle, np.empty((0, 2)), course["road_polygon"], final_reverse=True, timeout=2.)
    with (output/"reverse_parking_path.csv").open("w") as f:
        writer = csv.writer(f); writer.writerow(["x_m", "y_m", "yaw_rad", "direction"])
        writer.writerows((p.x, p.y, p.yaw, p.direction) for p in path)
    hold = ParkingHold()
    hold.update(0., Pose(), True, 0.)
    reset = not hold.update(5., Pose(.025, 0., 0.), True, 0.)
    result = {"synthetic_only": True, "red_stop_held_after_signal_loss": held_on_dropout,
              "confirmed_green_released": mission.signal_done, "stationary_hold_resets_on_motion": reset,
              "reverse_path_points": len(path), "reverse_path_collision_checked": path_clear(path, np.empty((0, 2)), vehicle, course["road_polygon"]),
              "real_vehicle_validated": False}
    (output/"summary.json").write_text(json.dumps(result, indent=2))
    if not all(result[k] for k in ("red_stop_held_after_signal_loss", "confirmed_green_released", "stationary_hold_resets_on_motion", "reverse_path_points", "reverse_path_collision_checked")):
        raise RuntimeError(result)
    return result
