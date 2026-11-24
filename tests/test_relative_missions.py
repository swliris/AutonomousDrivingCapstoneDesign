import json

import cv2
import numpy as np
import pytest

from second.cli import main
from second.config import validate
from second.demo import observation
from second.geometry import inverse
from second.localization import ScanOdometry
from second.markings import RoadMarks
from second.mission import Mission
from second.parking_scene import ParkingScene
from second.runtime import observation_from
from second.types import Pose


@pytest.fixture
def calibrated(setup):
    config, _, vehicle = setup
    config.update(calibrated=True, firmware_verified=True)
    config["calibration"]["steer_to_command"] = [[-vehicle.max_steer, 110], [0., 0], [vehicle.max_steer, -110]]
    for camera in config["cameras"].values():
        camera.update(width=640, height=480, intrinsics=[[500, 0, 320], [0, 500, 240], [0, 0, 1]],
                      distortion=[0.]*5, ground_homography=[[0, -.01, 4.6], [-.01, 0, 3.2], [0, 0, 1]])
    config["perception"]["signal"].update(roi=[0, 0, 1, 1], roi_verified=True)
    return config, vehicle


def ground_line(image, a, b, width=5):
    def pixel(point):
        x, y = point
        return round(320-100*y), round(460-100*x)
    cv2.line(image, pixel(a), pixel(b), 255, width)


def test_rolling_scan_reference_tracks_displacement_without_a_map():
    ref = np.random.default_rng(8).uniform(-3., 3., (350, 2))
    odom = ScanOdometry(.035)
    for i in range(25):
        truth = Pose(.025*i, -.007*i, .007*i)
        pose, ok = odom.update(inverse(ref, truth), 1.+.1*i)
        assert ok == (i != 0)
        assert np.allclose(pose.vector(), truth.vector(), atol=1e-5)
    assert not odom.update(ref[:10], 4.)[1]
    assert odom.stamp < 4.


@pytest.mark.parametrize("mode", ["mission1", "parking"])
def test_no_course_raw_cli(calibrated, tmp_path, mode):
    config, _ = calibrated
    assert not validate(config, None, mode)
    image = np.zeros((480, 640), np.uint8)
    for y in (-.425, .425, 1.275):
        ground_line(image, (.15, y), (4., y))
    cv2.imwrite(str(tmp_path/"frame.png"), image)
    angles = np.linspace(-np.pi, np.pi, 480, endpoint=False)
    ranges = 5 + .5*np.sin(3*angles)
    points = ranges[:, None]*np.column_stack((np.cos(angles), np.sin(angles)))
    np.savez(tmp_path/"scan.npz", points=points, angles=angles, ranges=ranges)
    (tmp_path/"sensor_config.json").write_text(json.dumps(config))
    rows = [dict(sensor=name, stamp=t, path="scan.npz" if name == "lidar" else "frame.png")
            for t in (1., 1.1) for name in ("lidar", "front", "rear")]
    (tmp_path/"frames.jsonl").write_text("".join(json.dumps(row)+"\n" for row in rows))
    output = tmp_path/"out.jsonl"
    assert main(["replay-raw", "--recording", str(tmp_path), "--mode", mode, "--output", str(output)]) == 0
    first, last = map(json.loads, output.read_text().splitlines())
    assert not first["localized"] and last["localized"]
    obs = observation_from(last)
    if mode == "mission1":
        assert set(obs.lanes) == {1, 2} and obs.scene["road_polygon"]
        mission = Mission(config, None, mode, calibrated[1])
        mission.start(1.)
        mission.state = "lane"
        obs.feedback_stamp = obs.stamp
        assert mission.update(obs).pwm > 0
    else:
        assert obs.scene["parking"]["perpendicular"] == {} and not obs.parked_valid


def test_crosswalk_distance_is_relative_and_plain_line_is_ignored(calibrated):
    config, vehicle = calibrated
    obs = observation(0.)
    marks = RoadMarks(config, vehicle)
    image = np.zeros((480, 640), np.uint8)
    ground_line(image, (1.4,-.9), (1.4,.9))
    marks.see_paint(image, obs.lanes, Pose())
    assert marks.stop_zone(Pose()) == (None, False)
    for y in np.arange(-.5,.6,.2):
        ground_line(image, (1.6,y), (2.5,y), 7)
    for _ in range(2):
        marks.see_paint(image, obs.lanes, Pose())
    d, inside = marks.stop_zone(Pose())
    assert abs(d-(1.6-vehicle.front-vehicle.margin)) < .08 and not inside
    d, inside = marks.stop_zone(Pose(1.6-vehicle.front-.10,0.,0.))
    assert inside and d < .13


def test_gate_posts_require_repeat_and_measured_crossing(calibrated):
    config, vehicle = calibrated
    marks = RoadMarks(config, vehicle)
    data = np.vstack([np.column_stack((np.full(7, 1.5), y+np.linspace(-.04,.04,7))) for y in (-.58,1.43)])
    lanes = observation(0.).lanes
    marks.see_posts(data, lanes, Pose(), 1.)
    assert marks.gate is None
    marks.see_posts(data, lanes, Pose(), 1.1)
    assert marks.gate is not None and np.allclose(marks.gate[:,0], 1.5)
    marks.see_posts(data[:7], lanes, Pose(), 3.)
    assert marks.gate is None


@pytest.mark.parametrize("kind,number,pitch,lip,back,index", [
    ("perpendicular","1",1.,-.65,-2.15,1), ("perpendicular","2",1.,-.65,-2.15,2),
    ("parallel","2",1.55,.65,1.60,1), ("parallel","1",1.55,.65,1.60,2)])
def test_parking_gap_from_camera_edges_and_two_reference_cars(calibrated,kind,number,pitch,lip,back,index):
    config, vehicle = calibrated
    parking = ParkingScene(config, vehicle)
    start, first = .55, 1.55
    ground = np.zeros((480,640),np.uint8)
    ground_line(ground, (start,-.95),(start,.95))
    scan = np.vstack([np.column_stack((np.linspace(x-.20,x+.20,15),np.full(15,(lip+back)/2)))
                      for x in (first+(index-.5)*pitch, first+(index+1.5)*pitch)])
    for car_x in np.arange(0., first+4*pitch, .45):
        frame = ground.copy() if car_x == 0 else np.zeros_like(ground)
        ground_line(frame,(first-car_x,back),(first+4*pitch-car_x,back))
        for x in first+pitch*np.arange(5):
            ground_line(frame,(x-car_x,lip),(x-car_x,back))
        for _ in range(2):
            parking.see_paint("front",frame,Pose(car_x,0.,0.),1.)
        park = parking.update(Pose(car_x,0.,0.),scan,1.)
    assert set(park[kind]) == {number}
    goal = park[kind][number]["goal"]
    assert first+index*pitch < goal[0] < first+(index+1)*pitch
    if kind=="parallel":
        assert len(park["parallel_entry_lines"]) == 4
        x = first+4*pitch+.6
        for _ in range(2):
            parking.dividers.add(x,-1.,1.)
        assert parking.update(Pose(),scan,1.)["out_line"][0][0] == x


def test_signal_sequence_and_same_gate_required_without_survey(calibrated):
    config, vehicle = calibrated
    mission = Mission(config, None, "mission1", vehicle)
    mission.start(0.)
    mission.state = "lane"
    def gate(t, x=None):
        scene = {"timer_line": [[x,-.5],[x,1.35]]} if x is not None else {}
        return mission.update(observation(t, scene=scene, gate=x is not None))
    gate(.1, 2.)
    gate(.2)
    gate(.3, 2.)
    assert mission.laps == 0
    mission.signal_done = True
    gate(.4)
    gate(.5, 8.)
    assert mission.laps == 0
    gate(.6)
    assert gate(.7, 2.1).state == "done"


def test_reverse_connections_reach_both_observed_goals(calibrated):
    from second.geometry import path_clear
    from second.planning import turning_paths
    config, vehicle = calibrated
    parking = ParkingScene(config, vehicle)
    allowed = [[-1.,-2.35], [8.,-2.35], [8.,1.8], [-1.,1.8]]
    for kind in ("perpendicular", "parallel"):
        perp = kind == "perpendicular"
        slot = parking.slot(2., 3. if perp else 3.55, -.85 if perp else .85, -2.35 if perp else 1.8, perp)
        paths = turning_paths(Pose(*slot["approach"]), Pose(*slot["goal"]), vehicle, -1)
        path = next(p for p in paths if path_clear(p, np.empty((0,2)), vehicle, allowed))
        end = path[-1]
        np.testing.assert_allclose([end.x, end.y, end.yaw], slot["goal"], atol=1e-7)
        assert all(p.direction == -1 for p in path)


def test_missing_road_or_parking_evidence_cannot_start_motion(calibrated):
    config, vehicle = calibrated
    for mode in ("mission1", "parking"):
        mission = Mission(config, None, mode, vehicle)
        mission.start(0.)
        if mode == "mission1":
            mission.state = "lane"
        obs = observation(.1, scene={})
        assert mission.update(obs).pwm == 0


def test_no_motion_on_stale_odometry(calibrated):
    config, vehicle = calibrated
    mission = Mission(config, None, "mission1", vehicle)
    mission.start(0.)
    mission.state = "lane"
    obs = observation(2., scene={"road_polygon":[[-3,-3],[7,-3],[7,3],[-3,3]]}, pose_stamp=1.)
    command = mission.update(obs)
    assert command.pwm == 0 and "stale" in command.reason


def test_sensor_corridor_keeps_current_lane_and_checks_escape(calibrated):
    from second.markings import road_corridor
    config, vehicle = calibrated
    obs = observation(.1)
    # An adjacent lane first seen beyond the bend must not erase our near lane.
    obs.lanes[1] = [p for p in obs.lanes[1] if p.x > 1.]
    area = road_corridor(obs.lanes, obs.pose, vehicle, .85)
    obs.scene = {"road_polygon": area}
    m = Mission(config, None, "mission1", vehicle)
    m.start(0.)
    m.state = "lane"
    assert m.update(obs).pwm > 0
    obs = observation(.2, obstacles=np.array([[1., 0.]]))
    obs.scene = {"road_polygon": road_corridor(obs.lanes, obs.pose, vehicle, .85)}
    assert m.update(obs).pwm == 0
    assert m.changing and m.destination_lane == 1


def test_aisle_search_needs_recognizable_markings(calibrated):
    config, vehicle = calibrated
    parking = ParkingScene(config, vehicle)
    mask = np.zeros((480,640), np.uint8)
    ground_line(mask, (1.,0.), (3.,.7))
    parking.see_paint("front", mask, Pose(), 1.)
    assert parking.update(Pose(), np.empty((0,2)), 1.)["search"] is None
