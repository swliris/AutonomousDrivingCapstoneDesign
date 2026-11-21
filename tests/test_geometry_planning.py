import math
import numpy as np
from second.geometry import contained, footprint, line_contact, parking_valid, path_clear, propagate, tire_polygons
from second.localization import Localizer
from second.planning import Tracker, hybrid_astar, lane_change, observed_free, with_headings
from second.types import Pose, Waypoint


def test_swept_body_sensor_overhang_and_margin_are_checked(setup):
    _, course, v = setup
    path = [Waypoint(0, 0), Waypoint(1, 0)]
    assert not path_clear(path, np.array([[.5, .1]]), v, course["road_polygon"])
    assert path_clear(path, np.array([[.5, 1.]]), v, course["road_polygon"])


def test_inner_hole_is_not_driveable(setup):
    _, _, v = setup
    outer = [[-3,-3],[3,-3],[3,3],[-3,3]]
    inner = [[-.1,-.1],[.1,-.1],[.1,.1],[-.1,.1]]
    assert not contained(footprint(Pose(), v), {"outer": outer, "holes": [inner]})


def test_perpendicular_sensor_front_must_be_inside_even_if_tires_fit(setup):
    _, course, v = setup
    slot = course["parking"]["perpendicular"]["1"]
    assert parking_valid(Pose(), v, slot, True)
    assert not parking_valid(Pose(.65, 0, 0), v, slot, True)


def test_tire_contact_not_body_contact_controls_out(setup):
    _, _, v = setup
    line = [[.4,-2],[.4,2]]
    assert line_contact([footprint(Pose(), v)], line)
    assert not line_contact(tire_polygons(Pose(), v), line)


def test_reverse_path_is_kinematically_feasible_and_collides_with_wall(setup):
    _, course, v = setup
    path = hybrid_astar(Pose(1.2, 0, 0), Pose(), v, np.empty((0,2)), course["road_polygon"], final_reverse=True)
    assert path and path[-1].direction == -1
    assert path_clear(path, np.empty((0,2)), v, course["road_polygon"])
    assert all(math.hypot(b.x-a.x,b.y-a.y) <= .031 for a,b in zip(path,path[1:]))
    wall = np.column_stack([np.full(101,.6), np.linspace(-5,5,101)])
    assert not hybrid_astar(Pose(1.2,0,0), Pose(), v, wall, course["road_polygon"], final_reverse=True, timeout=.1)


def test_reverse_pure_pursuit_and_direction_change_pause(setup):
    _, _, v = setup
    tracker = Tracker(v)
    tracker.set_path([Waypoint(0,0,0,-1),Waypoint(-.5,.1,0,-1)])
    steer, direction, _ = tracker.update(Pose(), -.1, 0)
    assert steer > 0 and direction == -1
    tracker.set_path([Waypoint(0,0,0,1),Waypoint(.2,0,0,1),Waypoint(.2,0,0,-1),Waypoint(0,0,0,-1)])
    assert tracker.update(Pose(.2,0,0), .1, 0)[1] == 0
    assert tracker.update(Pose(.2,0,0), 0, .1)[1] == 0
    assert tracker.update(Pose(.2,0,0), 0, .5)[1] == 0
    assert tracker.update(Pose(.2,0,0), 0, .6)[1] == -1


def test_unknown_lidar_sector_is_not_free(setup):
    _, _, v = setup
    path = with_headings([[0,0],[.5,0],[1,0]])
    assert not observed_free(path, Pose(), np.array([]), np.array([]), Pose(), v)
    assert not observed_free(path, Pose(), np.linspace(1.,2.,100), np.full(100,10.), Pose(), v)
    assert observed_free(path, Pose(), np.linspace(-math.pi,math.pi,1440), np.full(1440,10.), Pose(), v)


def test_lane_change_limits_and_target_visibility(setup):
    _, _, v = setup
    target = with_headings(np.column_stack([np.linspace(.1,4,80), np.full(80,.85)]))
    assert lane_change(Pose(), target, v, 3.)
    assert not lane_change(Pose(), target[:10], v, 3.)


def test_icp_recovers_metric_pose_from_static_structure():
    rng = np.random.default_rng(7)
    reference = rng.uniform(-2,2,(500,2))
    from second.geometry import inverse
    actual = Pose(.07,-.04,.025)
    localizer = Localizer(reference, Pose(), max_rmse=.03)
    pose, valid = localizer.update(inverse(reference,actual), 1.)
    assert valid and math.hypot(pose.x-actual.x,pose.y-actual.y) < .02
    assert abs(pose.yaw-actual.yaw) < .02
    assert not localizer.update(np.zeros((5,2)),2.)[1]
