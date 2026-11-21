"""Offline A/B measurement on the matched 640x480 recording and reviewed signal crops.

The fixed projection approximates perspective on the straight part of this recording.
It is evaluation-only. The polygon at row 420 substitutes for lane-ID association
with a surveyed map. No polygon pixels are used to find or fit paint tracks.
Score separate image rows. Flag clipped polygons and out-of-view predictions.
"""
import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

from second.config import ROOT, load_json
from second.perception import Vision
from second.types import Pose


ROWS = (380, 340, 300, 260)
ANCHOR = 420
BORDER_PX = 3  # Some annotations follow the image border a few pixels inside.
# In pixels, apparent lane width grows about 1.5 pixels per row below the horizon.
# X = 0.15 + 208/(v-100), Y = (320-u)*0.85/(1.5*(v-100)).
SCALE = 0.85 / 1.5
PROJECTION = [[0, .15, 193], [-SCALE, 0, 320*SCALE], [0, 1, -100]]


def save_csv(path, rows):
    with path.open('w', newline='', encoding='utf-8') as out:
        writer = csv.DictWriter(out, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)


def evaluate_lanes(vision, image_dir, labels):
    coco = json.loads(labels.read_text())
    lane_id = next(c['id'] for c in coco['categories'] if c['name'] == 'lane2')
    polygons = defaultdict(list)
    for ann in coco['annotations']:
        if ann['category_id'] == lane_id:
            polygons[ann['image_id']].extend(ann['segmentation'])
    results = []
    for im in sorted(coco['images'], key=lambda i: i['id']):
        image = cv2.imread(str(image_dir / im['file_name']))
        if image is None or image.shape[:2] != (480, 640):
            raise ValueError(f"expected a 640x480 frame: {im['file_name']}")
        region = np.zeros(image.shape[:2], np.uint8)
        for polygon in polygons[im['id']]:
            cv2.fillPoly(region, [np.rint(np.reshape(polygon, (-1, 2))).astype(np.int32)], 1)
        anchor = np.flatnonzero(region[ANCHOR])
        paths = {}
        single_span = anchor.size > 0 and np.all(np.diff(anchor) == 1)
        if single_span and not (anchor[0] <= BORDER_PX and anchor[-1] >= 639-BORDER_PX):
            # A clipped edge cannot determine the center. Estimate from the
            # visible opposite edge and the fixed reference width at the anchor.
            half = 1.5 * (ANCHOR - 100) / 2
            center = (anchor[0] + anchor[-1]) / 2
            if anchor[0] <= BORDER_PX:
                center = anchor[-1] - half
            elif anchor[-1] >= 639-BORDER_PX:
                center = anchor[0] + half
            paths = {'2': [[.8, (320 - center) * SCALE / (ANCHOR - 100)]]}
        lanes, _, _ = vision.lanes(image, Pose(), {'lane_paths': paths})
        path = lanes.get(2, [])
        for v in ROWS:
            pixels = np.flatnonzero(region[v])
            u = None
            x = .15 + 208 / (v - 100)
            if path and path[0].x <= x <= path[-1].x:
                y = np.interp(x, [p.x for p in path], [p.y for p in path])
                u = 320 - y * (v - 100) / SCALE
            left = int(pixels[0]) if len(pixels) else None
            right = int(pixels[-1]) if len(pixels) else None
            if left is None:
                visibility = 'absent'
            elif np.any(np.diff(pixels) > 1):
                visibility = 'multiple_spans'
            elif left <= BORDER_PX or right >= 639-BORDER_PX:
                visibility = 'clipped'
            else:
                visibility = 'full'
            if not paths:
                state = 'no_reference'
            elif visibility in ('absent', 'multiple_spans'):
                state = 'no_single_span'
            elif u is None:
                state = 'missing'
            elif not 0 <= u <= 639:
                state = 'offscreen'
            elif left <= u <= right:
                state = 'inside'
            else:
                state = 'outside'
            results.append(dict(frame=im['id'], file=im['file_name'], row=v, left=left,
                                right=right, visibility=visibility, center_px=u, state=state))
    return results


def evaluate_signals(vision, image_dir, reviewed):
    with reviewed.open() as f:
        rows = list(csv.DictReader(f))
    for row in rows:
        frame = cv2.imread(str(image_dir / row['image']))
        x, y, end_x, end_y = [int(row[k]) for k in ('x1', 'y1', 'x2', 'y2')]
        if frame is None or not (0 <= x < end_x <= frame.shape[1] and 0 <= y < end_y <= frame.shape[0]):
            raise ValueError(f"invalid reviewed signal image or box: {row['image']}")
        row['prediction'] = vision.light(frame[y:end_y, x:end_x]).value
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--images', required=True, type=Path)
    parser.add_argument('--labels', required=True, type=Path)
    parser.add_argument('--signal-images', required=True, type=Path)
    parser.add_argument('--reviewed-signals', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    cfg = load_json(ROOT / 'configs/vehicle.example.json')
    cfg['cameras']['front'].update(intrinsics=[[500, 0, 320], [0, 500, 240], [0, 0, 1]],
                                   distortion=[0.]*5, ground_homography=PROJECTION)
    cfg['perception']['signal'].update(roi=[0., 0., 1., 1.], roi_verified=True)
    vision = Vision(cfg)
    args.output.mkdir(parents=True, exist_ok=True)
    save_csv(args.output / 'lanes.csv', evaluate_lanes(vision, args.images, args.labels))
    save_csv(args.output / 'signals.csv', evaluate_signals(vision, args.signal_images, args.reviewed_signals))


if __name__ == '__main__':
    main()
