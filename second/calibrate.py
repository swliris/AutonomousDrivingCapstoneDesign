"""Offline calibration tools. These functions never connect to motor control."""
import json
from pathlib import Path
import cv2
import numpy as np
from .geometry import transform
from .types import Pose


def intrinsics(images, columns, rows, square_m, output):
    object_template = np.zeros((rows*columns, 3), np.float32)
    object_template[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)*square_m
    objects, pixels = [], []
    shape = None
    for path in images:
        image = cv2.imread(str(path))
        if image is None:
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        current_shape = gray.shape[::-1]
        if shape is not None and current_shape != shape:
            raise ValueError("all calibration images must have the same resolution")
        shape = current_shape
        found, corners = cv2.findChessboardCorners(gray, (columns, rows))
        if found:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                      (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 40, .001))
            objects.append(object_template)
            pixels.append(corners)
    if len(objects) < 12:
        raise ValueError(f"need at least twelve usable varied checkerboard views; found {len(objects)}")
    rmse, k, dist, rvecs, tvecs = cv2.calibrateCamera(objects, pixels, shape, None, None)
    result = {"intrinsics": k.tolist(), "distortion": dist.ravel().tolist(), "width": shape[0], "height": shape[1],
              "rms_reprojection_px": float(rmse), "usable_images": len(objects)}
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    Path(output).write_text(json.dumps(result, indent=2), encoding="utf-8")
    return result


def ground(points_file, output):
    points = json.loads(Path(points_file).read_text(encoding="utf-8"))
    image = np.asarray(points["undistorted_image_px"], float)
    body = np.asarray(points["rear_axle_xy_m"], float)
    if image.shape != body.shape or len(image) < 6 or image.shape[1] != 2:
        raise ValueError("need >=6 matched non-collinear points in UNDISTORTED image pixels and rear-axle metres")
    h, mask = cv2.findHomography(image, body, cv2.RANSAC, .025)
    if h is None or np.count_nonzero(mask) < 6:
        raise ValueError("homography fit failed")
    projected = cv2.perspectiveTransform(image[None].astype(np.float32), h)[0]
    residual = np.linalg.norm(projected-body, axis=1)
    if np.quantile(residual, .9) > .05:
        raise ValueError("ground calibration residual >5cm; remeasure correspondences")
    result = {"ground_homography": h.tolist(), "residual_m": residual.tolist(), "coordinate_frame": "rear axle: x forward, y left"}
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    Path(output).write_text(json.dumps(result, indent=2), encoding="utf-8")
    return result


def build_map(anchor_file, output):
    """Merge stationary scans taken at physically surveyed rear-axle poses.

    File: [{"scan":"data/run/lidar/0000000.npz", "pose":[x,y,yaw]}, ...].
    Use static structures only; remove people/movable objects in capture.
    """
    anchors = json.loads(Path(anchor_file).read_text(encoding="utf-8"))
    all_points = []
    for anchor in anchors:
        scan = np.load(anchor["scan"], allow_pickle=False)["points"]
        all_points.append(transform(scan, Pose(*anchor["pose"])))
    if not all_points:
        raise ValueError("no surveyed anchor scans")
    points = np.concatenate(all_points)
    points = points[np.isfinite(points).all(axis=1)]
    _, idx = np.unique(np.round(points/.025).astype(int), axis=0, return_index=True)
    points = points[idx]
    if len(points) < 100:
        raise ValueError("insufficient static map points")
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    np.save(output, points)
    return len(points)


def undistort_file(config, camera, source, output):
    image = cv2.imread(str(source))
    if image is None:
        raise ValueError("cannot read calibration image")
    c = config["cameras"][camera]
    if image.shape[:2] != (c["height"], c["width"]):
        raise ValueError("image dimensions differ from camera calibration")
    result = cv2.undistort(image, np.asarray(c["intrinsics"], float), np.asarray(c["distortion"], float))
    if not cv2.imwrite(str(output), result):
        raise IOError(output)
