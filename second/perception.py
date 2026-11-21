import cv2
import numpy as np
from .geometry import inverse, transform
from .planning import with_headings
from .types import Light


def _local_lane_reference(points, lookahead):
    """Choose a nearby point on the surveyed centerline in vehicle coordinates."""
    # Keep partial surveys ending in the matching window. Also intersect segments
    # with the lookahead so a sparsely surveyed straight needs no vertex there.
    nearby = points[(points[:, 0] > .3) & (points[:, 0] < 1.3)]
    starts, ends = points[:-1], points[1:]
    crosses = (starts[:, 0] < lookahead) != (ends[:, 0] < lookahead)
    starts, ends = starts[crosses], ends[crosses]
    fraction = (lookahead - starts[:, 0]) / (ends[:, 0] - starts[:, 0])
    crossings = starts + fraction[:, None] * (ends - starts)
    references = np.concatenate((nearby, crossings))
    if not len(references):
        return None
    distances = np.hypot(references[:, 0] - lookahead, references[:, 1])
    return references[np.argmin(distances)]


def _paint_tracks(xs, ys, sampled):
    """Match each paint hit once, preferring continuation along the recent tangent."""
    tracks = []
    row_step = xs[1] - xs[0]
    for x, row in zip(xs, sampled):
        indices = np.flatnonzero(row)
        groups = np.split(indices, np.flatnonzero(np.diff(indices) > 1) + 1)
        hits = [(float(ys[g].mean()), len(g)) for g in groups if 1 < len(g) < 16]
        matches = []
        for i, track in enumerate(tracks):
            last_x, last_y, last_width = track[-1]
            dx = x - last_x
            if dx >= .65:
                continue
            earlier_x, earlier_y, _ = track[max(0, len(track)-5)]
            slope = (last_y-earlier_y)/(last_x-earlier_x) if earlier_x < last_x else 0.
            predicted = last_y + max(-1.5, min(1.5, slope)) * dx
            for j, (y, width) in enumerate(hits):
                # In a gap, a thin parking-box line can steal a dashed lane
                # track before the next paint segment comes into view.
                error = abs(predicted-y)
                if dx > 1.5*row_step and error > .05 and min(width, last_width) < .5*max(width, last_width):
                    continue
                if error < min(.20, .045 + .65*dx):
                    matches.append((error + .12*dx, i, j))
        used_tracks, used_hits = set(), set()
        for _, i, j in sorted(matches):
            if i not in used_tracks and j not in used_hits:
                tracks[i].append((x, *hits[j]))
                used_hits.add(j)
                used_tracks.add(i)
        tracks.extend([[(x, *hit)] for j, hit in enumerate(hits) if j not in used_hits])
    return tracks


def _lane_candidates(xs, tracks, lane_width, *, visible_section=False):
    boundaries = [np.asarray(track)[:, :2] for track in tracks if len(track) >= 4
                  and (visible_section or track[0][0] <= 1.2) and track[-1][0] - track[0][0] >= .14]
    candidates = []
    for i, a in enumerate(boundaries):
        for b in boundaries[i+1:]:
            start = max(a[0, 0], b[0, 0])
            end = min(a[-1, 0], b[-1, 0])
            if end - start < .14 or max(a[-1, 0]-a[0, 0], b[-1, 0]-b[0, 0]) < .5:
                continue
            x = xs[(xs >= start) & (xs <= end)]
            side_a = np.interp(x, *a.T)
            side_b = np.interp(x, *b.T)
            mid = (side_a + side_b) / 2
            # A transverse scan cuts an oblique lane wider than its normal width.
            slope = np.gradient(mid, x)
            width = np.abs(side_a-side_b) / np.hypot(1, np.clip(slope, -1.2, 1.2))
            bad = np.flatnonzero((width < .7*lane_width) | (width > 1.4*lane_width))
            stop = int(bad[0]) if len(bad) else len(x)
            if stop < 4:
                continue
            x, mid = x[:stop], mid[:stop]
            # Only bridge the short gap at an end if the other marking continues.
            earlier = a if a[0, 0] < b[0, 0] else b
            head = xs[(xs < x[0]) & (xs >= max(earlier[0, 0], x[0]-.3))]
            offset = mid[0] - np.interp(x[0], *earlier.T)
            mid = np.r_[np.interp(head, *earlier.T) + offset, mid]
            x = np.r_[head, x]
            if not visible_section and x[0] > .95:
                continue
            if not visible_section and not len(bad):
                longer = a if a[-1, 0] > b[-1, 0] else b
                tail = xs[(xs > x[-1]) & (xs <= min(longer[-1, 0], x[-1]+.3))]
                offset = mid[-1] - np.interp(x[-1], *longer.T)
                mid = np.r_[mid, np.interp(tail, *longer.T) + offset]
                x = np.r_[x, tail]
            # A single quadratic may not describe the far bend. Keep the
            # supported near portion if it independently passes the same fit.
            for length in (len(x), len(x) // 2):
                x_fit, center = x[:length], mid[:length]
                if (not visible_section and x_fit[-1] < 1.15) or x_fit[-1]-x_fit[0] < .25:
                    break
                fit = np.polyfit(x_fit, center, 2 if length >= 12 else 1)
                if np.quantile(np.abs(center-np.polyval(fit, x_fit)), .8) > .065:
                    continue
                if visible_section:
                    # At an image edge, report the interior of the fit only.
                    # Its endpoints have no observations on one side for slope.
                    px = np.linspace(x_fit[1], x_fit[-2], 65)
                    candidates.append((px, np.polyval(fit, px)))
                    break
                px = np.linspace(.15, x_fit[-1], 65)
                near_slope = np.polyval(np.polyder(fit), x_fit[0])
                py = np.polyval(fit, np.maximum(px, x_fit[0])) + np.minimum(px-x_fit[0], 0)*near_slope
                candidates.append((px, py))
                break
    return candidates


def _sample_paint(mask, gray, image_from_ground, xx, yy):
    ground = np.stack([xx.ravel(), yy.ravel(), np.ones(xx.size)])
    uv = image_from_ground @ ground
    pixels = np.full_like(uv[:2], -1.)
    np.divide(uv[:2], uv[2:], out=pixels, where=np.abs(uv[2:]) > 1e-9)
    u, v = pixels.reshape(2, *xx.shape).astype(np.float32)
    sampled = cv2.remap(mask, u, v, cv2.INTER_NEAREST)
    if gray is not None:
        # Separate the narrow outer stripe when it merges with the pale floor.
        values = cv2.remap(gray, u, v, cv2.INTER_LINEAR)
        ridge = cv2.morphologyEx(values, cv2.MORPH_TOPHAT, np.ones((1, 21), np.uint8))
        broad = cv2.morphologyEx(sampled, cv2.MORPH_OPEN, np.ones((1, 17), np.uint8),
                                 borderType=cv2.BORDER_CONSTANT, borderValue=0)
        sampled[(broad > 0) & (ridge < 18)] = 0
    return sampled


def _edge_lane_sections(mask, gray, h, lane_width, angle_deg, lookahead):
    """Sample a fixed finer grid along a bend. Return only visible centerline sections."""
    xs = np.linspace(-.55, 2.95, 141)
    ys = np.linspace(-1.9, 1.9, 381)
    xx, yy = np.meshgrid(xs, ys, indexing="ij")
    angle = np.deg2rad(angle_deg)
    c, s = np.cos(angle), np.sin(angle)
    def vehicle_coordinates(x, y):
        return lookahead + c*(x-lookahead) - s*y, s*(x-lookahead) + c*y
    forward, lateral = vehicle_coordinates(xx, yy)
    sampled = _sample_paint(mask, gray, h, forward, lateral)
    sampled[(forward < .25) | (forward > 2.8)] = 0
    tracks = _paint_tracks(xs, ys, sampled)
    result = []
    for px, py in _lane_candidates(xs, tracks, lane_width, visible_section=True):
        x, y = vehicle_coordinates(px, py)
        # The controller and replay represent a lane as y(x). Do not interpolate
        # through a turn that folds back on itself.
        stop = np.flatnonzero(np.diff(x) <= 0)
        end = int(stop[0])+1 if len(stop) else len(x)
        xy = np.column_stack((x[:end], y[:end]))
        if len(xy) >= 2 and xy[-1, 0] - xy[0, 0] >= .2:
            result.append(xy)
    return result


def _match_visible_section(sections, reference, h, image_shape):
    x_ref, y_ref = reference
    best, best_score = None, np.inf
    for section in sections:
        x, y = section.T
        if x[0]-x_ref > .55 or x[-1] < x_ref+.35:
            continue
        j = min(np.searchsorted(x, x[0]+.12), len(x)-1)
        slope = (y[j]-y[0])/(x[j]-x[0])
        # Use the tangent only for association. The returned path starts at
        # the first visible sample, even when the map reference is nearer.
        ref_y = np.interp(x_ref, x, y) + min(0., x_ref-x[0])*slope
        error = abs(ref_y-y_ref)
        if x[0] > x_ref:
            # If the near tangent and the short chord disagree on the lane
            # ID, extrapolation around this bend is too uncertain to match.
            tangent = (y[1]-y[0]) / (x[1]-x[0])
            tangent_error = abs(y[0] + (x_ref-x[0])*tangent - y_ref)
            error = max(error, tangent_error)
        if abs(slope) > 3 or error >= .18 or y_ref*(y[-1]-ref_y) > 0:
            continue
        # Prefer a longer, closer observation among similarly aligned sections.
        score = error + .075*max(0., x[0]-x_ref) + .06*max(0., x_ref+.85-x[-1])
        if score < best_score:
            best, best_score = section, score
    if best is None:
        return None
    # Associate using the paint-supported fit, then clip the reported center
    # to the camera. Clipping first would change the association tangent.
    projected = cv2.perspectiveTransform(best[None], h)[0]
    height, width = image_shape
    valid = np.flatnonzero((projected[:, 0] >= 0) & (projected[:, 0] < width)
                           & (projected[:, 1] >= 0) & (projected[:, 1] < height))
    for group in np.split(valid, np.flatnonzero(np.diff(valid) > 1)+1):
        if len(group) >= 2 and best[group[-1], 0] - best[group[0], 0] >= .2:
            return best[group]
    return None


def _bloom_scores(hsv, masks, min_area):
    """Recover a washed-out lamp only when its bright core has a colored halo."""
    hue, saturation, value = cv2.split(hsv)
    core = ((saturation < 60) & (value >= 235)).astype(np.uint8)
    _, labels, stats, _ = cv2.connectedComponentsWithStats(core, connectivity=8)
    # A clipped green LED can expose cyan around its white core.
    bands = dict(masks)
    bands[Light.GREEN] = (saturation >= 100) & (value >= 130) & (hue > 35) & (hue <= 95)
    scores = {}
    height, width = hue.shape
    for idx, (x, y, w, h, area) in enumerate(stats[1:], 1):
        clipped = x == 0 or y == 0 or x+w == width or y+h == height
        aspect_limit = 4 if clipped else 2.5
        if area < min_area or area < .45*w*h or max(w, h) > aspect_limit * min(w, h):
            continue
        radius = max(2, round(min(w, h) / 4))
        rows = slice(max(0, y-radius), y+h+radius)
        cols = slice(max(0, x-radius), x+w+radius)
        blob = (labels[rows, cols] == idx).astype(np.uint8)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*radius+1, 2*radius+1))
        ring = (cv2.dilate(blob, kernel) > 0) & (blob == 0)
        ring_size = np.count_nonzero(ring)
        for color, band in bands.items():
            if np.count_nonzero(band[rows, cols] & ring) > ring_size / 2:
                scores[color] = max(scores.get(color, 0), int(area))
    return scores


class Vision:
    def __init__(self, config):
        self.config = config
        self.session = None
        self.detector = None
        model = config["perception"].get("segmentation_onnx")
        if model:
            import onnxruntime as ort
            providers = [p for p in ("CUDAExecutionProvider", "CPUExecutionProvider") if p in ort.get_available_providers()]
            self.session = ort.InferenceSession(model, providers=providers)
        signal_model = config["perception"]["signal"].get("model")
        if signal_model:
            from ultralytics import YOLO
            self.detector = YOLO(signal_model)

    def undistort(self, frame, name):
        c = self.config["cameras"][name]
        return cv2.undistort(frame, np.asarray(c["intrinsics"], float), np.asarray(c["distortion"], float))

    def mask(self, frame):
        if self.session:
            size = self.config["perception"]["model_size"]
            rgb = cv2.cvtColor(cv2.resize(frame, tuple(size)), cv2.COLOR_BGR2RGB).astype(np.float32)/255.
            rgb = (rgb-np.array([.485, .456, .406], np.float32))/np.array([.229, .224, .225], np.float32)
            tensor = rgb.transpose(2, 0, 1)[None]
            logits = self.session.run(None, {self.session.get_inputs()[0].name: tensor})[0]
            classes = logits[0].argmax(axis=0).astype(np.uint8)
            return cv2.resize((classes == 1).astype(np.uint8)*255, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)
        # Working baseline before training; explicitly white-paint only.
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        p = self.config["perception"]["white_threshold"]
        return cv2.inRange(hsv, np.array([0, 0, p["min_value"]]), np.array([179, p["max_saturation"], 255]))

    def lanes(self, frame, pose, course):
        frame = self.undistort(frame, "front")
        mask = self.mask(frame)
        gray = None if self.session else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h = np.linalg.inv(np.asarray(self.config["cameras"]["front"]["ground_homography"], float))
        xs = np.linspace(.25, 4.0, 76)
        ys = np.linspace(-2.0, 2.0, 401)
        xx, yy = np.meshgrid(xs, ys, indexing="ij")
        sampled = _sample_paint(mask, gray, h, xx, yy)
        lookahead = .8
        lane_width = self.config["perception"]["lane_width_m"]
        candidates = _lane_candidates(xs, _paint_tracks(xs, ys, sampled), lane_width)
        lanes = {}
        sections = {}
        for lane_id, map_path in course["lane_paths"].items():
            reference = _local_lane_reference(inverse(map_path, pose), lookahead)
            if reference is None:
                continue
            x_ref, y_ref = reference
            offsets = [abs(np.interp(x_ref, px, py) - y_ref) for px, py in candidates]
            if offsets and min(offsets) < .30:
                center = np.column_stack(candidates[np.argmin(offsets)])
            else:
                if abs(y_ref) < .4 * lane_width:
                    continue
                # Only relax near visibility when the outer lane boundary is
                # actually outside the camera horizontally at the reference.
                side = np.sign(y_ref)
                pixel = h @ [x_ref, y_ref + side*lane_width/2, 1.]
                if abs(pixel[2]) < 1e-9 or 0 <= pixel[0]/pixel[2] < frame.shape[1]:
                    continue
                for angle in (0., -side * 40.):
                    if angle not in sections:
                        sections[angle] = _edge_lane_sections(mask, gray, h, lane_width, angle, lookahead)
                center = _match_visible_section(sections[0.] + sections[-side*40.], reference, h, mask.shape)
                if center is None:
                    continue
                if center[0, 0] <= lookahead and center[0, 0] > .15:
                    # Same near tangent as the normal tracker, only when the
                    # recovered section actually reaches the camera lookahead.
                    x, y = center.T
                    j = min(np.searchsorted(x, x[0]+.12), len(x)-1)
                    near_x = np.arange(.15, x[0], .05)
                    near_y = y[0] + (near_x-x[0]) * (y[j]-y[0])/(x[j]-x[0])
                    center = np.vstack((np.column_stack((near_x, near_y)), center))
            lanes[int(lane_id)] = with_headings(transform(center, pose))
        return lanes, mask, frame

    def light(self, frame):
        signal = self.config["perception"]["signal"]
        roi = signal.get("roi")
        if self.detector:
            result = self.detector(frame, verbose=False, conf=signal["confidence"])[0]
            boxes = result.boxes
            if boxes is None or len(boxes) == 0:
                return Light.UNKNOWN
            # Only trained signal classes; never arbitrary COCO class IDs.
            candidates = []
            for box in boxes:
                name = str(result.names[int(box.cls.item())]).lower()
                if name in ("red", "green", "yellow", "traffic_light"):
                    candidates.append((float(box.conf.item()), name, box.xyxy[0].cpu().numpy()))
            if not candidates:
                return Light.UNKNOWN
            _, name, bbox = max(candidates, key=lambda b: b[0])
            if name in ("red", "green", "yellow"):
                return Light(name)
            x1, y1, x2, y2 = np.round(bbox).astype(int)
            crop = frame[max(0, y1):y2, max(0, x1):x2]
        elif roi and signal.get("roi_verified"):
            x, y, w, h = roi
            height, width = frame.shape[:2]
            crop = frame[int(y*height):int((y+h)*height), int(x*width):int((x+w)*width)]
        else:
            return Light.UNKNOWN
        if crop.size == 0:
            return Light.UNKNOWN
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        sat = (hsv[:, :, 1] >= 100) & (hsv[:, :, 2] >= 130)
        hue = hsv[:, :, 0]
        masks = {Light.RED: sat & ((hue < 12) | (hue > 167)), Light.GREEN: sat & (hue > 35) & (hue < 90), Light.YELLOW: sat & (hue >= 15) & (hue <= 35)}
        scores = {}
        for color, mask in masks.items():
            _, _, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
            scores[color] = max((int(s[cv2.CC_STAT_AREA]) for s in stats[1:] if .4 < s[cv2.CC_STAT_WIDTH]/max(1, s[cv2.CC_STAT_HEIGHT]) < 2.5), default=0)
        min_area = max(8, crop.shape[0] * crop.shape[1] * .004)
        if max(scores.values()) < min_area:
            for color, area in _bloom_scores(hsv, masks, min_area).items():
                scores[color] = max(scores[color], area)
        ranking = sorted(scores, key=scores.get, reverse=True)
        if scores[ranking[0]] < min_area or scores[ranking[0]] < 2*max(1, scores[ranking[1]]):
            return Light.UNKNOWN
        return ranking[0]

    def parking_alignment(self, frame, name, pose, slot):
        """Check surveyed painted bay edges against freshly observed paint.

        LiDAR determines pose; camera paint evidence validates the physical bay.
        Empty/invisible edges never establish success.
        """
        edges = slot.get("painted_edges", [])
        if not edges:
            return False
        image = self.undistort(frame, name)
        mask = self.mask(image)
        distance = cv2.distanceTransform(255-mask, cv2.DIST_L2, 3)
        inverse_h = np.linalg.inv(np.asarray(self.config["cameras"][name]["ground_homography"], float))
        visible_edges = 0
        for edge in edges:
            world = np.linspace(edge[0], edge[1], 50)
            local = inverse(world, pose)
            uv = inverse_h@np.column_stack([local, np.ones(len(local))]).T
            uv = (uv[:2]/uv[2:]).T
            valid = np.isfinite(uv).all(axis=1) & (uv[:, 0] >= 2) & (uv[:, 0] < image.shape[1]-2) & (uv[:, 1] >= 2) & (uv[:, 1] < image.shape[0]-2)
            pixels = np.round(uv[valid]).astype(int)
            if len(pixels) >= 10 and np.mean(distance[pixels[:, 1], pixels[:, 0]] < 6.) >= .60:
                visible_edges += 1
        return visible_edges >= 2
