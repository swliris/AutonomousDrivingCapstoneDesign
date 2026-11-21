"""Train a detector with dataset classes red,green,yellow,traffic_light.
Dataset split must be by recording session, not adjacent frames.
"""
import argparse


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--data", required=True, help="Ultralytics dataset yaml, measured track images")
    p.add_argument("--base", default="yolo11n.pt")
    p.add_argument("--epochs", type=int, default=50)
    p.add_argument("--project", default="models/signal")
    args = p.parse_args()
    from ultralytics import YOLO
    YOLO(args.base).train(data=args.data, epochs=args.epochs, imgsz=960, project=args.project, name="train", seed=7)


if __name__ == "__main__":
    main()
