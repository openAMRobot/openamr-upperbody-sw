# SPDX-License-Identifier: MIT
# Copyright (c) 2026 OpenAMRobot (Botshare LTD)
"""Test helpers: expand the upper-body xacro and compute frame poses."""

import itertools
import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import xacro
import yaml

PACKAGE = 'openamrobot_upperbody_description'
SHARE = get_package_share_directory(PACKAGE)
XACRO_FILE = os.path.join(SHARE, 'urdf', 'openamrobot_upperbody.urdf.xacro')

with open(os.path.join(SHARE, 'config', 'upperbody_configuration.yaml')) as _f:
    CONFIG = yaml.safe_load(_f)

AXES = CONFIG['configuration_axes']
ROOT = 'base_footprint'
EXPECTED_FRAMES = [
    'base_footprint', 'lift_column_base_link', 'lift_carriage_link',
    'arm_profile_link', 'openarm_left_base_link', 'openarm_right_base_link',
    'head_camera_mount_link', 'head_camera_link', 'head_camera_optical_frame',
]


def combinations():
    """Every column x base_plate x head_pitch x base_tilt combination."""
    names = ['column', 'base_plate', 'head_pitch', 'base_tilt']
    for values in itertools.product(*(AXES[n] for n in names)):
        yield dict(zip(names, values))


def buildable(combo):
    col = CONFIG['column'][combo['column']]
    return col['retracted_length'] is not None and col['stroke'] is not None


def expand(column_axis_x_bp000=None, **combo):
    """Expand the standalone model; column_axis_x_bp000=None keeps its xacro default."""
    mappings = dict(combo)
    if column_axis_x_bp000 is not None:
        mappings['column_axis_x_bp000'] = str(column_axis_x_bp000)
    return xacro.process_file(XACRO_FILE, mappings=mappings).toprettyxml(indent='  ')


def _rot(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _mul(a, b):
    """Compose 4x4 homogeneous transforms given as (R, t)."""
    ra, ta = a
    rb, tb = b
    r = [[sum(ra[i][k] * rb[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    t = [sum(ra[i][k] * tb[k] for k in range(3)) + ta[i] for i in range(3)]
    return r, t


def frame_poses(urdf_xml, joint_positions=None):
    """Return {link: (R, t)} relative to the root by forward kinematics."""
    joint_positions = joint_positions or {}
    root = ET.fromstring(urdf_xml)
    children = {}
    for j in root.findall('joint'):
        o = j.find('origin')
        xyz = [float(v) for v in (o.get('xyz') if o is not None else '0 0 0').split()]
        rpy = [float(v) for v in (o.get('rpy') if o is not None else '0 0 0').split()]
        children.setdefault(j.find('parent').get('link'), []).append(
            (j.get('name'), j.get('type'), j.find('child').get('link'), xyz, rpy,
             [float(v) for v in j.find('axis').get('xyz').split()]
             if j.find('axis') is not None else None))
    identity = ([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0.0, 0.0, 0.0])
    poses = {ROOT: identity}
    stack = [ROOT]
    while stack:
        parent = stack.pop()
        for name, jtype, child, xyz, rpy, axis in children.get(parent, []):
            pose = _mul(poses[parent], (_rot(*rpy), xyz))
            if jtype == 'prismatic':
                q = joint_positions.get(name, 0.0)
                pose = _mul(pose, (identity[0], [a * q for a in axis]))
            elif jtype not in ('fixed',):
                raise AssertionError(f'unexpected joint type {jtype} for {name}')
            poses[child] = pose
            stack.append(child)
    return poses
