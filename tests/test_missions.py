import numpy as np
import pytest
from second.demo import observation
from second.mission import Mission, ParkingHold
from second.planning import with_headings
from second.types import Command, Light, Pose, Waypoint


def machine(setup, mode="mission1"):
    config, course, v = setup
    m = Mission(config,course,mode,v)
    m.start(0.)
    return m


def test_no_motor_before_start_key(setup):
    config,course,v = setup
    m=Mission(config,course,"mission1",v)
    assert m.update(observation(1.,light=Light.GREEN)).pwm == 0
    assert m.state == "idle"


def test_one_green_frame_cannot_release_start(setup):
    m=machine(setup)
    for t in (.1,.2,.3):
        assert m.update(observation(t,light=Light.GREEN,light_stamp=.1)).pwm == 0
    assert m.state == "initial_green"


def test_red_latches_through_dropout_and_requires_fresh_green(setup):
    m=machine(setup); m.state="lane"; m.signal_left_start=True; m.first_motion=0.
    assert m.update(observation(.1,light=Light.RED,stop_zone_distance=.01,in_stop_zone=True)).pwm == 0
    for t in np.arange(.2,1.,.1):
        assert m.update(observation(float(t),light=Light.UNKNOWN,in_stop_zone=True)).pwm == 0
    assert m.state == "wait_green" and not m.signal_done
    for t in np.arange(1.,1.6,.1):
        m.update(observation(float(t),light=Light.GREEN,in_stop_zone=True))
    assert m.signal_done and m.state == "lane"


def test_green_outside_stop_zone_does_not_complete_signal(setup):
    m=machine(setup); m.state="wait_green"; m.red_latched=True; m.first_motion=0.
    for t in np.arange(.1,1.,.1):
        assert m.update(observation(float(t),light=Light.GREEN,in_stop_zone=False)).pwm == 0
    assert not m.signal_done


def test_stale_sensor_interrupts_lane_change(setup):
    m=machine(setup); m.state="lane"; m.changing=True
    c=m.update(observation(1.,front_stamp=0.))
    assert c.pwm == 0 and "stale" in c.reason


def test_hold_resets_on_motion_wrong_slot_and_heading_change():
    h=ParkingHold()
    assert not h.update(0,Pose(),True,0)
    assert not h.update(4.9,Pose(),True,0)
    assert not h.update(5,Pose(.02,0,0),True,0)
    assert not h.update(10,Pose(.02,0,.04),True,0)
    assert not h.update(15,Pose(.02,0,.04),False,0)
    assert not h.update(16,Pose(.02,0,.04),True,0)
    assert h.update(21.3,Pose(.02,0,.04),True,0)


def test_any_parallel_entry_irreversibly_ends_perpendicular(setup):
    m=machine(setup,"parking")
    m.parking_motion=lambda *args,**kwargs: Command()
    m.update(observation(.1,parallel_entry=True))
    assert m.state == "parallel_approach"
    assert not m.parking_results["perpendicular"]
    m.update(observation(.2,parallel_entry=False))
    assert m.state == "parallel_approach"


def test_wrong_target_and_forward_arrival_do_not_earn_parking(setup):
    m=machine(setup,"parking"); m.state="perpendicular_hold"; m.first_motion=0.
    m.reverse_seen=True
    for t in np.arange(0,6,.1):
        m.update(observation(float(t),parked_valid=True,parked_slot="perpendicular:2"))
    assert not m.parking_results["perpendicular"]
    m.reverse_seen=False
    for t in np.arange(6,12,.1):
        m.update(observation(float(t),parked_valid=True,parked_slot="perpendicular:1"))
    assert not m.parking_results["perpendicular"]


def test_both_parking_stages_then_out_without_restart(setup):
    m=machine(setup,"parking"); m.first_motion=0.
    m.state="perpendicular_hold"; m.reverse_seen=True
    for t in np.arange(0,5.5,.1):
        m.update(observation(float(t),parked_valid=True,parked_slot="perpendicular:1"))
    assert m.parking_results["perpendicular"]
    m.state="parallel_hold"; m.reverse_seen=True
    for t in np.arange(6,11.5,.1):
        m.update(observation(float(t),parked_valid=True,parked_slot="parallel:1"))
    assert all(m.parking_results.values())
    assert m.update(observation(12.,out_contact=True)).pwm == 0
    assert m.state == "done"


def test_timeout_preserves_already_completed_stage(setup):
    m=machine(setup,"parking"); m.parking_results["perpendicular"]=True
    m.update(observation(241.))
    assert m.state == "failed" and m.parking_results["perpendicular"]


def test_obstacle_causes_perception_based_lane_change(setup):
    setup[0]["control"]["lane_change_pwm"] = 137
    m=machine(setup); m.state="lane"; m.first_motion=0.
    c=m.update(observation(.1,speed=.1,obstacles=np.array([[1.,0.]])))
    assert m.changing and m.destination_lane == 1 and c.pwm == 0
    assert m.tracker.path
    assert m.update(observation(.2, speed=.1, obstacles=np.array([[1., 0.]]))).pwm == 137


@pytest.mark.parametrize("slope,expected_pwm", [(0., 193), (.22, 124), (-.22, 124)])
def test_lane_uses_configured_pwm_for_either_turn(setup, slope, expected_pwm):
    setup[0]["control"].update(straight_pwm=193, curve_pwm=124, curve_steer_deg=10)
    mission = machine(setup)
    mission.state = "lane"
    obs = observation(.1)
    x = np.linspace(.15, 3., 60)
    obs.lanes[2] = with_headings(np.column_stack((x, slope*x)))
    assert mission.update(obs).pwm == expected_pwm


@pytest.mark.parametrize("direction", [1, -1])
def test_parking_uses_signed_configured_pwm(setup, direction):
    setup[0]["control"]["parking_pwm"] = 121
    mission = machine(setup, "parking")
    mission.tracker.set_path([Waypoint(x*direction, 0., 0., direction) for x in np.linspace(0., 1., 20)])
    assert mission.update(observation(.1)).pwm == direction*121


def test_signal_approach_and_stop_use_lidar_speed(setup):
    setup[0]["control"].update(straight_pwm=180, curve_pwm=125)
    mission = machine(setup, "signal_only")
    mission.state = "lane"
    assert mission.update(observation(.1, stop_zone_distance=1., speed=.1)).pwm == 180
    assert mission.update(observation(.2, stop_zone_distance=.20, speed=.1)).pwm == 125
    assert mission.update(observation(.3, stop_zone_distance=.20, speed=.4)).pwm == 0
    assert mission.state == "wait_green"


def test_distant_lane_section_cannot_bridge_unobserved_approach(setup):
    m = machine(setup)
    m.state = "lane"
    m.first_motion = 0.
    obs = observation(.1)
    obs.lanes[2] = [p for p in obs.lanes[2] if p.x >= 1.]
    command = m.update(obs)
    assert command.pwm == 0 and command.reason == "current lane not observed"

    obs = observation(.2, obstacles=np.array([[1., 0.]]))
    obs.lanes[1] = [p for p in obs.lanes[1] if p.x >= 1.]
    command = m.update(obs)
    assert command.pwm == 0 and command.reason == "adjacent lane not observed"
    assert not m.changing
