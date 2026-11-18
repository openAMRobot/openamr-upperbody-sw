# SPDX-License-Identifier: MIT
"""A test fixture: the planar depth image of boxes seen by an ideal pinhole camera, by ray casting. No ROS import.

It stands in for the simulator's depth camera in the unit tests of box_fit.py. It is not a simulation of a camera:
no noise, no lens, no rendering artefact. Boxes are (centre xyz, size xyz, yaw) in the target frame.
"""
import math

import numpy as np


def optical_rotation(yaw, pitch_down):
    """Rotation of a REP-103 optical frame (z forward, x right, y down) whose camera body looks along `yaw` and is
    pitched down by `pitch_down` radians, in a z-up target frame."""
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch_down), math.sin(pitch_down)
    body = np.array([[cy * cp, -sy, cy * sp], [sy * cp, cy, sy * sp], [-sp, 0.0, cp]])   # Rz(yaw) Ry(pitch_down)
    optical_in_body = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    return body @ optical_in_body


def intrinsics(width, height, horizontal_fov):
    fx = width / (2.0 * math.tan(horizontal_fov / 2.0))
    return fx, fx, (width - 1) / 2.0, (height - 1) / 2.0


def render(boxes, rotation, translation, width, height, fx, fy, cx, cy, near=0.1, far=10.0):
    """Depth (height x width, metres along the optical axis, inf where nothing is hit) of `boxes`."""
    v, u = np.mgrid[0:height, 0:width]
    rays = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u, dtype=np.float64)], axis=-1).reshape(-1, 3)
    directions = rays @ np.asarray(rotation).T
    origin = np.asarray(translation, dtype=np.float64)
    depth = np.full(len(rays), np.inf)
    for centre, size, yaw in boxes:
        c, s = math.cos(-yaw), math.sin(-yaw)
        turn = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        p = turn @ (origin - np.asarray(centre, dtype=np.float64))
        d = directions @ turn.T
        half = np.asarray(size, dtype=np.float64) / 2.0
        with np.errstate(divide="ignore", invalid="ignore"):
            t1, t2 = (-half - p) / d, (half - p) / d
        t_near = np.nanmax(np.minimum(t1, t2), axis=1)
        t_far = np.nanmin(np.maximum(t1, t2), axis=1)
        hit = (t_far >= t_near) & (t_near > near) & (t_near < far)
        depth = np.where(hit & (t_near < depth), t_near, depth)
    return depth.reshape(height, width)


def tote_boxes(bottom_centre, size, wall, yaw):
    """The five boxes of an open tote with its pose at the bottom centre (as the Build 1 world builds it)."""
    sx, sy, sz = size
    parts = [((sx, sy, wall), (0.0, 0.0, wall / 2.0)),
             ((sx, wall, sz), (0.0, sy / 2.0 - wall / 2.0, sz / 2.0)),
             ((sx, wall, sz), (0.0, -sy / 2.0 + wall / 2.0, sz / 2.0)),
             ((wall, sy, sz), (-sx / 2.0 + wall / 2.0, 0.0, sz / 2.0)),
             ((wall, sy, sz), (sx / 2.0 - wall / 2.0, 0.0, sz / 2.0))]
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for part_size, (px, py, pz) in parts:
        out.append(((bottom_centre[0] + c * px - s * py, bottom_centre[1] + s * px + c * py, bottom_centre[2] + pz), part_size, yaw))
    return out
