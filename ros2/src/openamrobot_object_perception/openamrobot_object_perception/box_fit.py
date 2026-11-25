# SPDX-License-Identifier: MIT
"""Pose of known boxes from one depth image: a tote and an item standing in it. No ROS import.

Method (docs/architecture/object_perception.md, milestone 1). The depth pixels become points in the robot frame
(base_footprint) through the camera intrinsics and the camera's pose from /tf. A band of heights above the shelf level
that the task names holds each object's visible faces. The face that looks at the camera is a vertical plane: a RANSAC
line through the band's points, seen from above, gives its direction (the yaw) and its offset, and the known box size
gives the centre behind it. Along the face, the centre is the middle of the face's extent; the box top gives the height.

Inputs that are known before the robot looks: the box sizes (the item catalogue) and the height of the shelf level (the
site map). No pose of any object is an input. The estimates are for a noise-free simulated depth camera; nothing here
is a measurement of a real camera, and nothing here is a safety function.
"""
import math
from dataclasses import dataclass, field

import numpy as np

# Variance (rad^2) given to roll and pitch: the objects are assumed upright and these are not estimated.
UNESTIMATED_VARIANCE = 1.0


@dataclass
class Estimate:
    """Pose of one object in the points' frame. `position` is the box centre for a box and the bottom centre for a
    tote (the same convention as the simulator's model poses). `yaw` is about +z of that frame. `covariance` is the
    6 x 6 row-major covariance of (x, y, z, roll, pitch, yaw), measurement-resolution terms only (Proposed)."""
    name: str
    position: tuple
    yaw: float
    covariance: list
    points: int
    detail: dict = field(default_factory=dict)


def quaternion_matrix(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def wrap(angle, period):
    """`angle` folded into [-period / 2, period / 2)."""
    return (angle + period / 2.0) % period - period / 2.0


def symmetry(size):
    """Yaw period of a box of `size`: a quarter turn for a square footprint, a half turn otherwise."""
    return math.pi / 2.0 if abs(size[0] - size[1]) < 1e-6 else math.pi


def depth_points(depth, fx, fy, cx, cy, rotation, translation, stride=1, max_range=3.0):
    """Points (N x 3) in the target frame of a planar depth image (metres along the optical axis, REP-103 optical
    frame). `rotation` (3 x 3) and `translation` (3) are the optical frame's pose in the target frame."""
    d = np.asarray(depth, dtype=np.float64)[::stride, ::stride]
    rows, cols = d.shape
    v, u = np.mgrid[0:rows, 0:cols]
    u, v = u * stride, v * stride
    ok = np.isfinite(d) & (d > 0.0) & (d < max_range)
    z = d[ok]
    camera = np.stack([(u[ok] - cx) * z / fx, (v[ok] - cy) * z / fy, z], axis=1)
    return camera @ np.asarray(rotation, dtype=np.float64).T + np.asarray(translation, dtype=np.float64)


def in_region(points, region):
    """Points inside `region` (x_min, x_max, y_min, y_max) of the target frame."""
    x_min, x_max, y_min, y_max = region
    keep = (points[:, 0] >= x_min) & (points[:, 0] <= x_max) & (points[:, 1] >= y_min) & (points[:, 1] <= y_max)
    return points[keep]


def ransac_line(xy, tolerance, iterations=300, seed=0):
    """The 2D line with the most points within `tolerance`. -> (unit normal, offset c with normal . p = c, inlier mask)
    or None. Seeded: the same points give the same line. The line is refined by a total least squares fit."""
    count = len(xy)
    if count < 2:
        return None
    rng = np.random.default_rng(seed)
    best, best_count = None, 0
    for _ in range(iterations):
        i, j = rng.choice(count, 2, replace=False)
        d = xy[j] - xy[i]
        length = math.hypot(d[0], d[1])
        if length < 0.005:
            continue
        normal = np.array([-d[1], d[0]]) / length
        inliers = np.abs(xy @ normal - normal @ xy[i]) < tolerance
        found = int(inliers.sum())
        if found > best_count:
            best, best_count = inliers, found
    if best is None:
        return None
    for _ in range(2):
        chosen = xy[best]
        centre = chosen.mean(axis=0)
        _, _, vt = np.linalg.svd(chosen - centre, full_matrices=False)
        normal = np.array([-vt[0][1], vt[0][0]])
        c = float(normal @ centre)
        best = np.abs(xy @ normal - c) < tolerance
        if best.sum() < 2:
            return None
    return normal, c, best


def grid_clusters(xy, cell):
    """Connected groups of points on a grid of `cell` metres (8-neighbour). -> label per point."""
    if len(xy) == 0:
        return np.zeros(0, dtype=np.int64)
    keys = np.floor(xy / cell).astype(np.int64)
    unique, inverse = np.unique(keys, axis=0, return_inverse=True)
    index = {(int(k[0]), int(k[1])): i for i, k in enumerate(unique)}
    label = np.full(len(unique), -1, dtype=np.int64)
    current = 0
    for start in range(len(unique)):
        if label[start] >= 0:
            continue
        label[start] = current
        stack = [start]
        while stack:
            k = stack.pop()
            kx, ky = int(unique[k][0]), int(unique[k][1])
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    j = index.get((kx + dx, ky + dy))
                    if j is not None and label[j] < 0:
                        label[j] = current
                        stack.append(j)
        current += 1
    return label[np.asarray(inverse).ravel()]


def _facing(normal, c, inlier_xy, camera_xy):
    """Normal and offset turned so that the normal points away from the camera, into the object."""
    if normal @ (inlier_xy.mean(axis=0) - np.asarray(camera_xy, dtype=np.float64)) < 0.0:
        return -normal, -c
    return normal, c


def _covariance(normal, sigma_n, sigma_t, sigma_z, sigma_yaw):
    frame = np.array([[normal[0], -normal[1]], [normal[1], normal[0]]])   # columns: normal, tangent
    xy = frame @ np.diag([sigma_n ** 2, sigma_t ** 2]) @ frame.T
    cov = np.zeros((6, 6))
    cov[:2, :2] = xy
    cov[2, 2] = sigma_z ** 2
    cov[3, 3] = cov[4, 4] = UNESTIMATED_VARIANCE
    cov[5, 5] = sigma_yaw ** 2
    return [float(v) for v in cov.ravel()]


def _resolution(points, camera_position, fx):
    """Size (m) of one pixel at the mean range of `points`."""
    return float(np.linalg.norm(points.mean(axis=0) - np.asarray(camera_position, dtype=np.float64))) / fx


def _yaw_sigma(residual_rms, pixel, inliers, extent):
    return max(residual_rms, pixel / math.sqrt(12.0)) * math.sqrt(12.0 / max(inliers, 1)) / max(extent, 1e-3)


def inside_footprint(xy, centre, size, yaw, margin=0.0):
    """True when the point `xy` lies in the rectangle of `size` (x, y) turned by `yaw` about `centre`, grown by `margin`."""
    dx, dy = xy[0] - centre[0], xy[1] - centre[1]
    c, s = math.cos(-yaw), math.sin(-yaw)
    return abs(c * dx - s * dy) <= size[0] / 2.0 + margin and abs(s * dx + c * dy) <= size[1] / 2.0 + margin


def estimate_item(points, size, rest_z, band_low, camera_position, fx, min_points=40, height_tolerance=0.012,
                  width_tolerance=0.01, face_tolerance=0.0015, name="item", container=None):
    """A box of `size` (x, y, z) standing on a surface at height `rest_z` (the tote floor), its top above `band_low`
    (the tote rim). -> (Estimate or None, reason). Points between `band_low` and 0.03 m above the expected top are
    grouped; a group is the item when it stops below the band top (a post or a column continues), its top is within
    `height_tolerance` of rest_z + size z (a lifted or fallen item is not taken), it fits inside the item's diagonal and
    its camera-facing face is as wide as one side of the item.

    `container` is ((x, y), (size x, size y), yaw) of the tote the item stands in, from the tote estimate of the same
    image. A group whose centre lies outside it is not the item: a shelf post is 0.04 m square and as tall as the band,
    and without this it was taken for a 0.035 m item in 9 of 315 parking poses on ideal depth (review of 25 Nov 2026)."""
    expected_top = rest_z + size[2]
    band_high = expected_top + 0.03
    band = points[(points[:, 2] > band_low) & (points[:, 2] < band_high)]
    if len(band) < min_points:
        return None, f"{len(band)} points between {band_low:.3f} and {band_high:.3f} m"
    labels = grid_clusters(band[:, :2], 0.01)
    half_diagonal = math.hypot(size[0], size[1]) / 2.0
    candidates, rejected = [], []
    for label in np.unique(labels):
        group = band[labels == label]
        if len(group) < min_points:
            continue
        top = float(group[:, 2].max())
        spread = float(np.max(np.linalg.norm(group[:, :2] - group[:, :2].mean(axis=0), axis=1)))
        if container is not None and not inside_footprint(group[:, :2].mean(axis=0), *container):
            rejected.append("outside the tote")
        elif top > band_high - 0.008:
            rejected.append("taller than the item")
        elif abs(top - expected_top) > height_tolerance:
            rejected.append(f"top {top:.3f} m, expected {expected_top:.3f} m")
        elif spread > half_diagonal + 0.01:
            rejected.append(f"spread {spread:.3f} m")
        else:
            candidates.append(group)
    if not candidates:
        return None, "no group fits the item: " + ("; ".join(rejected[:4]) or "none large enough")
    group = max(candidates, key=len)
    top = float(group[:, 2].max())
    top_points = group[group[:, 2] >= top - 0.002]
    sides = group[group[:, 2] < top - 0.006]
    if len(sides) < 15:
        return None, f"{len(sides)} side points"
    line = ransac_line(sides[:, :2], face_tolerance)
    if line is None or line[2].sum() < 15:
        return None, "no face line"
    normal, c, inliers = line
    face = sides[inliers]
    normal, c = _facing(normal, c, face[:, :2], camera_position[:2])
    tangent = np.array([-normal[1], normal[0]])
    along = np.concatenate([face[:, :2] @ tangent, top_points[:, :2] @ tangent])
    t_min, t_max = float(along.min()), float(along.max())
    width = t_max - t_min
    if abs(width - size[1]) <= abs(width - size[0]):     # the box's y side faces the camera: its x axis is the normal
        expected_width, depth, yaw = size[1], size[0], math.atan2(normal[1], normal[0])
    else:
        expected_width, depth, yaw = size[0], size[1], math.atan2(tangent[1], tangent[0])
    if abs(width - expected_width) > width_tolerance:
        return None, f"face {width:.4f} m wide, expected {expected_width:.4f} m"
    xy = normal * (c + depth / 2.0) + tangent * (t_min + t_max) / 2.0
    z = float(top_points[:, 2].mean()) - size[2] / 2.0
    pixel = _resolution(group, camera_position, fx)
    residual = float(np.sqrt(np.mean((face[:, :2] @ normal - c) ** 2)))
    cov = _covariance(normal, pixel / math.sqrt(12.0), pixel / math.sqrt(24.0), pixel / math.sqrt(12.0),
                      _yaw_sigma(residual, pixel, len(face), width))
    detail = {"face_points": int(len(face)), "top_points": int(len(top_points)), "face_width_m": round(width, 5),
              "top_m": round(top, 5), "face_residual_m": round(residual, 6), "pixel_m": round(pixel, 5),
              "groups": len(candidates)}
    return Estimate(name, (float(xy[0]), float(xy[1]), z), wrap(yaw, symmetry(size)), cov, int(len(group)), detail), "ok"


def estimate_tote(points, size, support_z, camera_position, fx, min_points=200, face_tolerance=0.002, run_gap=0.025,
                  length_tolerance=0.03, name="tote"):
    """An open box of outer `size` (x, y, z) standing on a shelf level at `support_z`. -> (Estimate or None, reason).
    Points of the band from 0.012 m above the level to just above the rim are taken; the face that looks at the camera
    is the nearest strong RANSAC line of that band. Along it, the run of points as long as one side of the tote (posts
    and other boxes on the same line are separate runs) gives the centre along the face; the pose is at the bottom."""
    top = support_z + size[2]
    band = points[(points[:, 2] > support_z + 0.012) & (points[:, 2] < top + 0.006)]
    if len(band) < min_points:
        return None, f"{len(band)} points between {support_z + 0.012:.3f} and {top + 0.006:.3f} m"
    xy = band[:, :2]
    line = ransac_line(xy, face_tolerance, iterations=400)
    if line is None:
        return None, "no face line"
    normal, c, inliers = line
    normal, c = _facing(normal, c, xy[inliers], camera_position[:2])
    for _ in range(3):   # a strong parallel line nearer to the camera is the front face (the inner back wall is farther)
        nearer = (xy @ normal - c) < -0.02
        if nearer.sum() < max(30, 0.5 * inliers.sum()):
            break
        found = ransac_line(xy[nearer], face_tolerance, iterations=400, seed=1)
        if found is None:
            break
        n2, c2, in2 = found
        if in2.sum() < 0.5 * inliers.sum() or abs(float(n2 @ normal)) < math.cos(0.2):
            break
        normal, c = _facing(n2, c2, xy[nearer][in2], camera_position[:2])
        inliers = np.abs(xy @ normal - c) < face_tolerance
    tangent = np.array([-normal[1], normal[0]])
    face = band[inliers]
    along = face[:, :2] @ tangent
    order = np.argsort(along)
    along_sorted = along[order]
    breaks = np.where(np.diff(along_sorted) > run_gap)[0]
    starts = np.concatenate([[0], breaks + 1])
    ends = np.concatenate([breaks, [len(along_sorted) - 1]])
    best = None
    for start, end in zip(starts, ends):
        length = float(along_sorted[end] - along_sorted[start])
        for axis in (0, 1):
            miss = abs(length - size[axis])
            if miss <= length_tolerance and (best is None or miss < best[0]):
                best = (miss, start, end, axis, length)
    if best is None:
        runs = [round(float(along_sorted[e] - along_sorted[s]), 3) for s, e in zip(starts, ends)]
        return None, f"no run of the face line as long as a tote side: runs {runs[:6]} m"
    _miss, start, end, axis, length = best
    run = face[order[start:end + 1]]
    t_mid = float(along_sorted[start] + along_sorted[end]) / 2.0
    depth = size[1 - axis]
    centre = normal * (c + depth / 2.0) + tangent * t_mid
    rim = float(run[:, 2].max())
    z = float(run[run[:, 2] >= rim - 0.002][:, 2].mean()) - size[2]
    yaw = math.atan2(tangent[1], tangent[0]) if axis == 0 else math.atan2(normal[1], normal[0])
    pixel = _resolution(run, camera_position, fx)
    residual = float(np.sqrt(np.mean((run[:, :2] @ normal - c) ** 2)))
    cov = _covariance(normal, pixel / math.sqrt(12.0), pixel / math.sqrt(24.0), pixel / math.sqrt(12.0),
                      _yaw_sigma(residual, pixel, len(run), length))
    detail = {"face_points": int(len(run)), "face_length_m": round(length, 5), "rim_m": round(rim, 5),
              "face_residual_m": round(residual, 6), "pixel_m": round(pixel, 5), "runs": int(len(starts))}
    return Estimate(name, (float(centre[0]), float(centre[1]), z), wrap(yaw, symmetry(size)), cov, int(len(run)), detail), "ok"
