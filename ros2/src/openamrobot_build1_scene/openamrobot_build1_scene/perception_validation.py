# SPDX-License-Identifier: MIT
"""Estimate minus ground truth of an object pose, in base_footprint. No ROS import.

Under object_pose_source estimate the scenarios and the planning scene take the estimator's poses; Gazebo's poses of the
objects and of the base are read here only to measure how far off the estimate is. Nothing computed here feeds a grasp
target, a planning scene or a command. Simulation evidence only.
"""
import math

import numpy as np

# Yaw period of each object's shape: the item's footprint is square, the tote's is a rectangle.
SYMMETRY = {"item": math.pi / 2.0, "tote": math.pi, "payload": math.pi}


def _matrix(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def truth_in_base(base_position, base_quaternion, position, quaternion):
    """An object's world pose (position, quaternion) in base_footprint, from the base's world pose. -> ((x, y, z), yaw)."""
    rotation = _matrix(base_quaternion)
    local = rotation.T @ (np.asarray(position, dtype=float) - np.asarray(base_position, dtype=float))
    return tuple(float(v) for v in local), math.atan2(math.sin(yaw_of(quaternion) - yaw_of(base_quaternion)),
                                                      math.cos(yaw_of(quaternion) - yaw_of(base_quaternion)))


def wrap(angle, period):
    return (angle + period / 2.0) % period - period / 2.0


def error(name, estimate_position, estimate_yaw, truth_position, truth_yaw, covariance=None):
    """Estimate minus truth, both in base_footprint. -> dict of dx, dy, dz, position (3D), horizontal, yaw (folded by
    the shape's symmetry) and, with a 6 x 6 row-major covariance, nees_xy: the horizontal error squared in units of the
    stated covariance (about 2 on average when the covariance is right)."""
    d = [float(e - t) for e, t in zip(estimate_position, truth_position)]
    out = {"dx": d[0], "dy": d[1], "dz": d[2], "position": math.sqrt(sum(v * v for v in d)), "horizontal": math.hypot(d[0], d[1]),
           "yaw": wrap(estimate_yaw - truth_yaw, SYMMETRY.get(name, 2.0 * math.pi))}
    if covariance is not None:
        cov = np.asarray(covariance, dtype=float).reshape(6, 6)[:2, :2]
        try:
            out["nees_xy"] = float(np.asarray(d[:2]) @ np.linalg.solve(cov, np.asarray(d[:2])))
        except np.linalg.LinAlgError:
            out["nees_xy"] = None
    return out


class Record:
    """Per object: the number of estimates measured, the last error, and the mean and largest position and yaw error."""

    def __init__(self):
        self.objects = {}

    def add(self, name, measured):
        entry = self.objects.setdefault(name, {"count": 0, "sum_position": 0.0, "max_position": 0.0, "max_abs_yaw": 0.0})
        entry["count"] += 1
        entry["sum_position"] += measured["position"]
        entry["max_position"] = max(entry["max_position"], measured["position"])
        entry["max_abs_yaw"] = max(entry["max_abs_yaw"], abs(measured["yaw"]))
        entry["last"] = {key: (round(value, 6) if isinstance(value, float) else value) for key, value in measured.items()}

    def summary(self):
        return {name: {"count": e["count"], "mean_position_m": round(e["sum_position"] / e["count"], 6),
                       "max_position_m": round(e["max_position"], 6), "max_abs_yaw_rad": round(e["max_abs_yaw"], 6),
                       "last": e["last"]} for name, e in self.objects.items()}
