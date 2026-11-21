import cv2
import numpy as np
import pytest

from second.config import ROOT, load_json, validate
from second.geometry import inverse, transform
from second.perception import Vision, _match_visible_section
from second.runtime import Observer
from second.types import Pose


@pytest.fixture
def camera_config():
    config = load_json(ROOT / "configs/vehicle.example.json")
    for camera in config["cameras"].values():
        camera.update(
            intrinsics=[[500, 0, 320], [0, 500, 240], [0, 0, 1]],
            distortion=[0.] * 5,
            ground_homography=[[0, -1/85, 440/85], [-1/110, 0, 320/110], [0, 0, 1]],
        )
    return config


@pytest.mark.parametrize("closed, vertices", [(False, 71), (True, 71), (True, 2)])
def test_lane_ids_on_near_straight(camera_config, closed, vertices):
    image = np.zeros((480, 640, 3), np.uint8)
    for boundary_y in (-.425, .425, 1.275):
        column = round(320 - 110 * boundary_y)
        cv2.line(image, (column, 435), (column, 85), (255, 255, 255), 6)

    pose = Pose(3., -2., .6)
    lane_paths = {}
    xs = np.linspace(-3., 4., vertices)
    for lane_id, y in ((1, .85), (2, 0.)):
        path = [(x, y) for x in xs]
        if closed:
            path.extend((x, 6. - y) for x in xs[::-1])
            path.append(path[0])
        lane_paths[str(lane_id)] = transform(path, pose)

    vision = Vision(camera_config)
    course = {"lane_paths": lane_paths}
    lanes, _, _ = vision.lanes(image, pose, course)
    assert set(lanes) == {1, 2}
    for lane_id, expected_y in ((1, .85), (2, 0.)):
        local = inverse([(p.x, p.y) for p in lanes[lane_id]], pose)
        np.testing.assert_allclose(local[:, 1], expected_y, atol=.03)

    # Erasing the left boundary leaves visual evidence for lane 2 only.
    image[:, :200] = 0
    lanes, _, _ = vision.lanes(image, pose, course)
    assert set(lanes) == {2}


def test_parking_first_observation_with_empty_road_stop_zone(camera_config, setup, tmp_path):
    _, course, _ = setup
    course["stop_zone"] = load_json(ROOT / "configs/course.template.json")["stop_zone"]
    course["parking"]["starts"] = {str(i): [0., 0., 0.] for i in range(1, 5)}
    course["parking"]["parallel_entry_lines"] = [[[2., -1.], [2., 1.]]]
    course["lidar_map"] = str(tmp_path / "map.npy")
    np.save(course["lidar_map"], np.zeros((40, 2)))
    course["surveyed"] = True
    camera_config.update(calibrated=True, firmware_verified=True)
    assert validate(camera_config, course, "parking") == []

    # Put the front tires on OUT to check that the parking observer ran.
    wheelbase = camera_config["vehicle"]["wheelbase"]
    pose = Pose(3. - wheelbase, 0., 0.)
    obs = Observer(camera_config, course, "parking", pose).observe(None, None, None, None)
    assert obs.out_contact and not obs.outside
    assert not obs.in_stop_zone and obs.stop_zone_distance is None


@pytest.mark.parametrize("mode", ["mission1", "signal_only"])
def test_road_observer_still_measures_stop_zone(setup, tmp_path, mode):
    config, course, _ = setup
    course["lidar_map"] = str(tmp_path / "map.npy")
    np.save(course["lidar_map"], np.zeros((40, 2)))
    course["stop_zone"] = {"inner_polygon": course["road_polygon"], "target_pose": [.6, 0., 0.]}
    obs = Observer(config, course, mode, Pose()).observe(None, None, None, None)
    assert obs.in_stop_zone
    assert obs.stop_zone_distance == pytest.approx(.6)


def test_short_dash_beside_bright_floor(camera_config):
    image = np.full((480, 640, 3), 65, np.uint8)
    # The floor beyond the right border also passes the white paint threshold.
    image[:, 368:] = 180
    cv2.line(image, (367, 435), (367, 100), (240, 240, 240), 6)
    # Only about 0.25 m of the near left dash is visible.
    cv2.line(image, (273, 374), (273, 354), (240, 240, 240), 5)
    course = {'lane_paths': {'2': [[.8, 0.0]]}}
    vision = Vision(camera_config)
    lanes, _, _ = vision.lanes(image, Pose(), course)
    assert 2 in lanes
    assert max(abs(point.y) for point in lanes[2]) < .04
    assert 1.15 <= lanes[2][-1].x < 1.4

    image[:, :320] = 65
    assert vision.lanes(image, Pose(), course)[0] == {}


@pytest.mark.parametrize('hue,expected', [(0, 'red'), (91, 'green'), (115, 'unknown')])
def test_signal_with_clipped_bloom(camera_config, hue, expected):
    camera_config['perception']['signal'].update(roi=[0., 0., 1., 1.], roi_verified=True)
    hsv = np.full((48, 160, 3), (hue, 240, 220), np.uint8)
    cv2.ellipse(hsv, (65, 1), (33, 25), 0, 0, 360, (0, 10, 255), -1)
    assert Vision(camera_config).light(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)).value == expected


def test_signal_needs_color_around_bright_core(camera_config):
    camera_config['perception']['signal'].update(roi=[0., 0., 1., 1.], roi_verified=True)
    frame = np.full((50, 160, 3), 35, np.uint8)
    cv2.circle(frame, (30, 25), 17, (255, 255, 255), -1)
    vision = Vision(camera_config)
    assert vision.light(frame).value == 'unknown'
    frame[:] = (0, 0, 230)  # Wide red housing with no illuminated core.
    assert vision.light(frame).value == 'unknown'


@pytest.mark.parametrize("side", [-1, 1])
def test_clipped_lane_recovered_only_over_visible_paint(camera_config, side):
    # Render a lane entering either edge of the image. The solid boundary is
    # off camera near the map reference. Farther ahead both boundaries appear.
    h = np.array([[0., .15, 193.], [-.85/1.5, 0., 320*.85/1.5], [0., 1., -100.]])
    camera_config["cameras"]["front"]["ground_homography"] = h.tolist()
    image = np.full((480, 640, 3), 65, np.uint8)
    slope = -.85 * side
    offset = .85 * side
    def center_y(x):
        return offset + slope*(x-.8)

    pose = Pose(2., -1., .6)
    course = {"lane_paths": {"2": transform([[.8, offset]], pose)}}
    vision = Vision(camera_config)
    for boundary in (1, -1):
        spans = [(0.68, 2.45)] if boundary == 1 else [(.74, .97), (1.05, 1.27), (1.4, 1.64), (1.78, 2.05)]
        for start, end in spans:
            x = np.array([start, end])
            y = center_y(x) + boundary * side * .425 * np.hypot(1, slope)
            uv = cv2.perspectiveTransform(np.column_stack((x, y))[None], np.linalg.inv(h))[0]
            cv2.line(image, tuple(np.rint(uv[0]).astype(int)), tuple(np.rint(uv[1]).astype(int)),
                     (240, 240, 240), 6)
        lanes, _, _ = vision.lanes(image, pose, course)
        if boundary == 1:
            assert not lanes

    local = inverse([(point.x, point.y) for point in lanes[2]], pose)
    assert .8 < local[0, 0] < 1.35
    assert local[-1, 0] > 1.5
    np.testing.assert_allclose(local[:, 1], center_y(local[:, 0]), atol=.045)

def test_partial_curve_with_ambiguous_near_tangent_is_not_assigned():
    # Both curves share a usable chord toward the map anchor. Only the straight
    # one's local tangent confirms that association across the unobserved gap.
    x = np.linspace(1.07, 1.5, 65)
    bent_y = -.35 + .45 * (1-np.exp(-10*(x-x[0])))
    j = np.searchsorted(x, x[0]+.12)
    slope = (bent_y[j]-bent_y[0])/(x[j]-x[0])
    straight_y = bent_y[0] + (x-x[0])*slope
    ref = (.8, bent_y[0] + (.8-x[0])*slope)
    h = np.array([[20., 0., 100.], [0., 20., 100.], [0., 0., 1.]])
    straight = np.column_stack((x, straight_y))
    bend = np.column_stack((x, bent_y))
    assert _match_visible_section([bend], ref, h, (480, 640)) is None
    np.testing.assert_allclose(_match_visible_section([straight], ref, h, (480, 640)), straight)
