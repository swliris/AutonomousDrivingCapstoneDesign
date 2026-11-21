import json

import cv2
import numpy as np
import pytest

from second import runtime
from second.cli import main
from second.compare_icp import compare
from second.geometry import inverse
from second.localization import _neighbor_query
from second.recording import Recording
from second.sensors import Sample
from second.types import Pose


@pytest.fixture
def recorded_run(setup, tmp_path):
    config, course, _ = setup
    for camera in config["cameras"].values():
        camera.update(width=640, height=480, intrinsics=[[500, 0, 320], [0, 500, 240], [0, 0, 1]],
                      distortion=[0.] * 5, ground_homography=[[0, -1/85, 440/85], [-1/110, 0, 320/110], [0, 0, 1]])
    course["road_starts"] = {"M3": [0., 0., 0.]}
    course["parking"]["starts"] = {"1": [0., 0., 0.]}
    course["parking"]["parallel_entry_lines"] = []
    # Two short, painted edges fall within the synthetic rear camera view.
    for slot in course["parking"]["perpendicular"].values():
        slot["painted_edges"] = [[[.3, -.4], [1., -.4]], [[.3, .4], [1., .4]]]
    course["lidar_map"] = str(tmp_path / "map.npy")
    points = np.random.default_rng(4).uniform(-3., 3., (200, 2))
    np.save(course["lidar_map"], points)
    course_file = tmp_path / "course.json"
    course_file.write_text(json.dumps(course))
    (tmp_path / "sensor_config.json").write_text(json.dumps(config))
    image = np.zeros((480, 640, 3), np.uint8)
    for y in (-.425, .425, 1.275):
        column = round(320 - 110*y)
        cv2.line(image, (column, 435), (column, 85), (255, 255, 255), 6)
    assert cv2.imwrite(str(tmp_path / "front.jpg"), image)
    assert cv2.imwrite(str(tmp_path / "rear.jpg"), np.full_like(image, 255))
    scan = {"points": points, "angles": np.arctan2(points[:, 1], points[:, 0]),
            "ranges": np.linalg.norm(points, axis=1)}
    np.savez_compressed(tmp_path / "lidar.npz", **scan)
    # The polling order differs from capture order. Front and LiDAR share one timestamp.
    events = [("front", 31., "front.jpg"), ("rear", 30.9, "rear.jpg"),
              ("lidar", 31., "lidar.npz"), ("front", 32., "front.jpg")]
    (tmp_path / "frames.jsonl").write_text("".join(json.dumps(dict(sensor=s, stamp=t, path=p)) + "\n"
                                                  for s, t, p in events))
    return config, course, course_file, scan


@pytest.mark.parametrize("mode,start", [("mission1", "M3"), ("parking", "1")])
def test_raw_cli_uses_live_observer_and_capture_clock(recorded_run, tmp_path, monkeypatch, mode, start):
    config, course, course_file, scan = recorded_run
    def no_device(*args):
        pytest.fail("offline replay touched hardware or GUI")

    monkeypatch.setattr(runtime, "SensorHub", no_device)
    monkeypatch.setattr(runtime, "Actuator", no_device)
    monkeypatch.setattr(cv2, "imshow", no_device)
    output = tmp_path / "out.jsonl"
    assert main(["--course", str(course_file), "replay-raw", "--recording", str(tmp_path),
                 "--mode", mode, "--start", start, "--output", str(output)]) == 0
    rows = [json.loads(line) for line in output.read_text().splitlines()]
    assert [r["stamp"] for r in rows] == [30.9, 31., 32.]
    assert not rows[0]["localized"] and rows[0]["front_stamp"] < 0
    assert rows[1]["localized"] and rows[1]["lidar_stamp"] == rows[2]["lidar_stamp"] == 31.
    if mode == "mission1":
        assert set(rows[1]["lanes"]) == {"1", "2"}
    else:
        assert rows[1]["parked_valid"] and not rows[2]["parked_valid"]

    # Feed the decoded samples through the live entry point, with its monotonic clock.
    observer = runtime.Observer(config, course, mode, Pose())
    front = cv2.imread(str(tmp_path / "front.jpg"))
    rear = Sample(30.9, cv2.imread(str(tmp_path / "rear.jpg")))
    lidar = Sample(31., scan)
    for row, front_sample, lidar_sample in zip(rows, [None, Sample(31., front), Sample(32., front)], [None, lidar, lidar]):
        monkeypatch.setattr(runtime.time, "monotonic", lambda: row["stamp"])
        obs = observer.observe(front_sample, rear, lidar_sample, None)
        assert row == runtime.observation_dict(obs)


def test_recorded_icp_and_mount_override(recorded_run, tmp_path, capsys):
    config, _, course_file, _ = recorded_run
    assert main(["--course", str(course_file), "compare-icp", "--recording", str(tmp_path),
                 "--repeats", "1"]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["scans"] == report["both_accepted"] == 1
    assert report["max_xy_delta_m"] == pytest.approx(0.)
    assert "xy_rmse_m" not in report["kdtree"]
    recording = Recording(tmp_path)
    config["lidar"]["angle_sign"] *= -1
    config["lidar"]["pose"] = [1., 0., 0.]
    scan = recording.scan(tmp_path / "lidar.npz", config)
    original = recording.scan(tmp_path / "lidar.npz", recording.config)
    np.testing.assert_allclose(scan["points"][:, 0], original["points"][:, 0] + 1.)
    np.testing.assert_allclose(scan["points"][:, 1], -original["points"][:, 1])


def test_bad_camera_frame_leaves_no_completed_log(recorded_run, tmp_path):
    _, _, course_file, _ = recorded_run
    assert cv2.imwrite(str(tmp_path / "front.jpg"), np.zeros((10, 10, 3), np.uint8))
    output = tmp_path / "out.jsonl"
    with pytest.raises(ValueError, match="resolution"):
        main(["--course", str(course_file), "replay-raw", "--recording", str(tmp_path), "--output", str(output)])
    assert not output.exists()


def test_numbered_pngs_use_numeric_order_and_real_mask(tmp_path):
    for index, value in [(10, 0), (2, 255)]:
        assert cv2.imwrite(str(tmp_path / f"{index}_steer:0.png"), np.full((20, 30, 3), value, np.uint8))
    output = tmp_path / "mask.jsonl"
    args = ["replay-images", "--input", str(tmp_path), "--output", str(output)]
    assert main(args) == 0
    rows = [json.loads(line) for line in output.read_text().splitlines()]
    assert [(r["frame"], r["paint_pixels"]) for r in rows] == [(2, 600), (10, 0)]
    with pytest.raises(FileExistsError):
        main(args)


def test_exact_neighbor_ties_keep_first_survey_point():
    ref = np.random.default_rng(13).uniform(5., 8., (50, 2))
    ref[[25, 17, 4]] = [[1., 0.], [0., -1.], [-1., 0.]]
    queries = np.array([[0., 0.], [.9, .1]])
    dense = _neighbor_query(ref, "dense")(queries)
    tree = _neighbor_query(ref, "kdtree")(queries)
    np.testing.assert_array_equal(tree[1], [4, 25])
    np.testing.assert_allclose(tree[0], dense[0])


def test_paired_icp_trajectory_rejects_dropout_and_recovers():
    reference = np.random.default_rng(42).uniform(-2., 2., (250, 2))

    def scans():
        for i in range(6):
            truth = Pose(.02 * i, -.01 * i, .005 * i)
            points = inverse(reference, truth)
            yield i * .1, points[:5] if i == 3 else points, truth

    report = compare(reference, Pose(), scans, .035, repeats=2)
    assert report["both_accepted"] == 5
    assert report["validity_mismatches"] == 0
    assert report["max_xy_delta_m"] < 1e-10
    assert report["max_yaw_delta_deg"] < 1e-8
    for backend in ("dense", "kdtree"):
        assert report[backend]["xy_rmse_m"] < 1e-6
        assert report[backend]["yaw_rmse_deg"] < 1e-4
        assert report[backend]["p95_ms"] >= report[backend]["median_ms"] > 0
