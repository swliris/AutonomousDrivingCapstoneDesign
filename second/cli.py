import argparse
from dataclasses import asdict
import glob
import json
import logging
from pathlib import Path
import sys
from .config import ROOT, load_json, validate, vehicle_from
from .mission import Mission


def main(argv=None):
    p = argparse.ArgumentParser(description="Second: mission-only two-camera/LiDAR vehicle software")
    p.add_argument("--config", help="vehicle JSON; defaults to the recording's sensor_config.json for raw recordings")
    p.add_argument("--course", default=str(ROOT/"configs/course.template.json"))
    sub = p.add_subparsers(dest="command", required=True)
    sub.add_parser("devices", help="list stable video/serial paths without opening motors")
    d = sub.add_parser("doctor", help="read-only live readiness checks")
    d.add_argument("--mode", choices=Mission.MODES, default="mission1")
    r = sub.add_parser("record", help="record two cameras + LiDAR, never Arduino")
    r.add_argument("--output", required=True)
    r.add_argument("--seconds", type=float, default=60.)
    for name in ("live", "replay", "replay-raw", "compare-icp"):
        q = sub.add_parser(name)
        q.add_argument("--mode", choices=Mission.MODES, default="mission1")
        q.add_argument("--start", default="M3", help="M3 road; parking 1,2,3,4")
        if name in ("live", "replay"):
            q.add_argument("--perpendicular", choices=("1", "2"), default="1")
            q.add_argument("--parallel", choices=("1", "2"), default="1")
        if name == "live":
            q.add_argument("--log", required=True)
        elif name == "replay":
            q.add_argument("--input", required=True)
            q.add_argument("--output", required=True)
        elif name == "replay-raw":
            q.add_argument("--recording", required=True, help="directory from the record command")
            q.add_argument("--output", required=True, help="new observations JSONL file")
            q.add_argument("--view", action="store_true", help="display cameras and LiDAR at recorded timestamps")
        else:
            source = q.add_mutually_exclusive_group(required=True)
            source.add_argument("--recording", help="compare the recorded LiDAR scans")
            source.add_argument("--synthetic", action="store_true", help="use a seeded trajectory with known poses")
            q.add_argument("--repeats", type=int, default=3)
    images = sub.add_parser("replay-images", help="replay a numbered camera-only image folder in pixel space")
    images.add_argument("--input", required=True)
    images.add_argument("--output", required=True, help="new per-image mask statistics JSONL file")
    images.add_argument("--view", action="store_true", help="display the source and paint mask")
    images.add_argument("--fps", type=float, default=10., help="display rate for images without timestamps")
    demo = sub.add_parser("demo", help="synthetic rules/planning smoke run; no hardware")
    demo.add_argument("--output", default=str(ROOT/"logs/demo"))
    c = sub.add_parser("calibrate-camera")
    c.add_argument("--images", required=True, help="quoted image glob")
    c.add_argument("--columns", type=int, required=True)
    c.add_argument("--rows", type=int, required=True)
    c.add_argument("--square-m", type=float, required=True)
    c.add_argument("--output", required=True)
    c = sub.add_parser("calibrate-ground")
    c.add_argument("--points", required=True)
    c.add_argument("--output", required=True)
    c = sub.add_parser("build-map")
    c.add_argument("--anchors", required=True)
    c.add_argument("--output", required=True)
    c = sub.add_parser("undistort")
    c.add_argument("--camera", choices=("front", "rear"), required=True)
    c.add_argument("--image", required=True)
    c.add_argument("--output", required=True)
    args = p.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    course = load_json(args.course)
    recording = None
    if getattr(args, "recording", None):
        from .recording import Recording
        recording = Recording(args.recording)
    if args.config is None and recording is not None:
        config = recording.config
    else:
        config = load_json(args.config or ROOT/"configs/vehicle.example.json")
    if args.command == "devices":
        print(json.dumps({"video_by_id": glob.glob("/dev/v4l/by-id/*"), "video_by_path": glob.glob("/dev/v4l/by-path/*"), "serial_by_id": glob.glob("/dev/serial/by-id/*")}, indent=2))
    elif args.command == "doctor":
        errors = validate(config, course, args.mode)
        for camera in config["cameras"].values():
            if not Path(camera["device"]).exists():
                errors.append(f"camera device not present: {camera['device']}")
        for name in ("serial", "lidar"):
            if not Path(config[name]["port"]).exists():
                errors.append(f"{name} device not present: {config[name]['port']}")
        print(json.dumps({"ready": not errors, "issues": errors}, ensure_ascii=False, indent=2))
        return int(bool(errors))
    elif args.command == "record":
        from .runtime import record
        print(record(config, args.output, args.seconds))
    elif args.command == "live":
        errors = validate(config, course, args.mode)
        if errors:
            p.error("live blocked:\n"+"\n".join(errors))
        valid_starts = course["parking"]["starts"] if args.mode == "parking" else course["road_starts"]
        if args.start not in valid_starts:
            p.error(f"choose a surveyed start from {list(valid_starts)}")
        from .runtime import live
        result = live(config, course, args.mode, args.start, args.perpendicular, args.parallel, args.log)
        print(json.dumps({"state": result.state, "events": result.events, "parking": result.parking_results}, ensure_ascii=False, indent=2))
    elif args.command == "replay":
        from .runtime import observation_from
        mission = Mission(config, course, args.mode, vehicle_from(config), args.perpendicular, args.parallel)
        count = 0
        with open(args.input, encoding="utf-8") as src, open(args.output, "w", encoding="utf-8") as out:
            for line in src:
                row = json.loads(line)
                obs = observation_from(row)
                # Recorded operator start is an explicit event; do not start on idle log frames.
                start = row.get("start", False) or any(e["code"] == "start_key" and e["time"] <= obs.stamp for e in row.get("events", []))
                if mission.state == "idle" and start:
                    mission.start(obs.stamp)
                command = mission.update(obs)
                out.write(json.dumps({"stamp": obs.stamp, "command": {k: v for k, v in asdict(command).items() if k != "path"}, "events": mission.events}, ensure_ascii=False)+"\n")
                count += 1
        print(json.dumps({"frames": count, "state": mission.state, "events": mission.events}, ensure_ascii=False, indent=2))
    elif args.command == "replay-raw":
        from .raw_replay import replay_raw
        result = replay_raw(recording, config, course, args.mode, args.start, args.output, args.view)
        print(json.dumps(result, ensure_ascii=False, indent=2))
    elif args.command == "replay-images":
        from .raw_replay import replay_images
        result = replay_images(args.input, config, args.output, args.view, args.fps)
        print(json.dumps(result, ensure_ascii=False, indent=2))
    elif args.command == "compare-icp":
        from .compare_icp import compare, recorded_case, synthetic_case
        case = synthetic_case() if args.synthetic else recorded_case(recording, config, course, args.mode, args.start)
        result = compare(*case, config["safety"]["icp_max_rmse_m"], args.repeats)
        result["source"] = "synthetic" if args.synthetic else args.recording
        print(json.dumps(result, ensure_ascii=False, indent=2))
    elif args.command == "demo":
        from .demo import run
        print(json.dumps(run(args.output), ensure_ascii=False, indent=2))
    elif args.command == "calibrate-camera":
        from .calibrate import intrinsics
        print(intrinsics(sorted(glob.glob(args.images)), args.columns, args.rows, args.square_m, args.output))
    elif args.command == "calibrate-ground":
        from .calibrate import ground
        print(ground(args.points, args.output))
    elif args.command == "build-map":
        from .calibrate import build_map
        print({"points": build_map(args.anchors, args.output)})
    elif args.command == "undistort":
        from .calibrate import undistort_file
        undistort_file(config, args.camera, args.image, args.output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
