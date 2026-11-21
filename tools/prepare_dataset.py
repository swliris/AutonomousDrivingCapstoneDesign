"""Extract spaced frames; masks are supplied by humans, never fabricated."""
import argparse
import json
from pathlib import Path
import shutil


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--recording", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--split", choices=("train", "val", "test"), required=True)
    p.add_argument("--interval", type=float, default=.5)
    args = p.parse_args()
    run = Path(args.recording).resolve()
    root = Path(args.output).resolve()
    (root/"images").mkdir(parents=True, exist_ok=True)
    (root/"masks").mkdir(exist_ok=True)
    manifest = root/"manifest.jsonl"
    existing = [json.loads(line) for line in manifest.read_text().splitlines()] if manifest.exists() else []
    if any(row["run_id"] == run.name for row in existing):
        raise ValueError("this run was already extracted; never split one run across train/val/test")
    last = {"front": -1e30, "rear": -1e30}
    count = 0
    with (run/"frames.jsonl").open() as frames, manifest.open("a") as out:
        for line in frames:
            row = json.loads(line)
            sensor = row["sensor"]
            if sensor not in last or row["stamp"]-last[sensor] < args.interval:
                continue
            last[sensor] = row["stamp"]
            source = run/row["path"]
            name = f"{run.name}_{sensor}_{source.stem}"
            image, mask = root/"images"/(name+".jpg"), root/"masks"/(name+".png")
            shutil.copy2(source, image)
            out.write(json.dumps({"image": str(image), "mask": str(mask), "run_id": run.name, "camera": sensor, "split": args.split})+"\n")
            count += 1
    print(f"Extracted {count} images. Label masks with integer classes 0,1,2,3 before training.")


if __name__ == "__main__":
    main()
