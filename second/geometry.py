import math
import numpy as np
from .types import Pose, Vehicle, Waypoint, wrap


def transform(points, pose: Pose):
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return p @ np.array([[c, s], [-s, c]]) + [pose.x, pose.y]


def inverse(points, pose: Pose):
    p = np.asarray(points, dtype=float).reshape(-1, 2) - [pose.x, pose.y]
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return p @ np.array([[c, -s], [s, c]])


def rectangle(front, rear, width):
    return np.array([[front, width/2], [-rear, width/2],
                     [-rear, -width/2], [front, -width/2]])


def footprint(pose: Pose, v: Vehicle, margin=0.):
    return transform(rectangle(v.front+margin, v.rear+margin, v.width+2*margin), pose)


def tire_polygons(pose: Pose, v: Vehicle, steer=0.):
    result = []
    # Conservative rectangular contact envelopes, front wheel rotates with steering.
    for x in (0., v.wheelbase):
        for y in (-v.track/2, v.track/2):
            tire = rectangle(v.tire_length/2, v.tire_length/2, v.tire_width)
            local = transform(tire, Pose(x, y, steer if x else 0.))
            result.append(transform(local, pose))
    return result


def inside(points, polygon, tolerance=1e-8):
    """Boundary inclusive point-in-polygon, for arbitrary simple polygons."""
    p = np.asarray(points, float).reshape(-1, 2)
    poly = np.asarray(polygon, float)
    answer = np.zeros(len(p), dtype=bool)
    boundary = np.zeros(len(p), dtype=bool)
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        ab = b-a
        ap = p-a
        cross = ab[0]*ap[:, 1]-ab[1]*ap[:, 0]
        boundary |= (np.abs(cross) <= tolerance) & ((ap @ ab) >= -tolerance) & ((ap @ ab) <= ab@ab+tolerance)
        if abs(ab[1]) > tolerance:
            hits = ((a[1] > p[:, 1]) != (b[1] > p[:, 1]))
            x = a[0] + (p[:, 1]-a[1])*ab[0]/ab[1]
            answer ^= hits & (p[:, 0] < x)
    return answer | boundary


def polygon_samples(poly, spacing=.02):
    points = []
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        n = max(2, int(np.linalg.norm(b-a)/spacing)+1)
        points.extend(np.linspace(a, b, n))
    return np.asarray(points)


def contained(shape, polygon):
    # Also sample edges: vertices alone are insufficient in a concave region.
    if isinstance(polygon, dict):
        if not contained(shape, polygon["outer"]):
            return False
        for hole in polygon.get("holes", []):
            if np.any(inside(polygon_samples(shape), hole)) or np.any(inside(hole, shape)):
                return False
        return True
    return bool(np.all(inside(polygon_samples(shape), polygon)))


def collides(pose: Pose, points, v: Vehicle, margin=None):
    if len(points) == 0:
        return False
    m = v.margin if margin is None else margin
    p = inverse(points, pose)
    return bool(np.any((p[:, 0] >= -v.rear-m) & (p[:, 0] <= v.front+m)
                       & (np.abs(p[:, 1]) <= v.width/2+m)))


def propagate(pose: Pose, distance, steer, wheelbase):
    k = math.tan(steer)/wheelbase
    if abs(k) < 1e-8:
        return Pose(pose.x+distance*math.cos(pose.yaw), pose.y+distance*math.sin(pose.yaw), pose.yaw)
    yaw = pose.yaw+distance*k
    return Pose(pose.x+(math.sin(yaw)-math.sin(pose.yaw))/k,
                pose.y+(-math.cos(yaw)+math.cos(pose.yaw))/k, wrap(yaw))


def segment_intersects(a, b, c, d):
    def cross(u, v):
        return u[0]*v[1]-u[1]*v[0]
    r, s = np.asarray(b)-a, np.asarray(d)-c
    den = cross(r, s)
    if abs(den) < 1e-10:
        return False
    t, u = cross(np.asarray(c)-a, s)/den, cross(np.asarray(c)-a, r)/den
    return 0 <= t <= 1 and 0 <= u <= 1


def line_contact(polygons, line):
    a, b = np.asarray(line, float)
    return any(np.any(inside([a, b], poly)) or
               any(segment_intersects(c, d, a, b) for c, d in zip(poly, np.roll(poly, -1, axis=0)))
               for poly in polygons)


def path_clear(path: list[Waypoint], points, v: Vehicle, allowed=None):
    if not path:
        return False
    for a, b in zip(path, path[1:]+path[-1:]):
        n = max(1, int(math.hypot(b.x-a.x, b.y-a.y)/.025)+1)
        for t in np.linspace(0, 1, n):
            p = Pose(a.x+t*(b.x-a.x), a.y+t*(b.y-a.y), wrap(a.yaw+t*wrap(b.yaw-a.yaw)))
            if collides(p, points, v) or (allowed is not None and not contained(footprint(p, v, v.margin), allowed)):
                return False
    return True


def parking_valid(pose, v, slot, perpendicular, steer=0., buffer=.02):
    """Strict inner region avoids the rule's allowed-but-penalised line contact."""
    polygon = slot["inner_polygon"]
    if not all(contained(poly, polygon) for poly in tire_polygons(pose, v, steer)):
        return False
    if perpendicular and not contained(footprint(pose, v, buffer), polygon):
        return False
    return True
