# SPDX-License-Identifier: MIT
# Copyright (c) 2026 OpenAMRobot (Botshare LTD)
"""Publish each buildable configuration with robot_state_publisher and look up
every frame in TF at lift 0 and full lift."""

import os
import signal
import subprocess
import tempfile
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
import yaml

from upperbody_model import CONFIG, EXPECTED_FRAMES, ROOT, buildable, combinations, expand

TOL = 1e-6
BUILDABLE = [c for c in combinations() if buildable(c)]


@pytest.fixture(scope='module')
def node():
    rclpy.init()
    n = Node('upperbody_tf_probe')
    n.tf_buffer = Buffer()
    n.tf_listener = TransformListener(n.tf_buffer, n)
    n.joint_pub = n.create_publisher(JointState, '/joint_states', 10)
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _start_rsp(urdf):
    params = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    yaml.safe_dump({'robot_state_publisher': {'ros__parameters': {
        'robot_description': urdf}}}, params)
    params.close()
    return subprocess.Popen(
        ['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
         '--ros-args', '--params-file', params.name],
        stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, start_new_session=True)


def _stop(proc):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=5)


def _lookup_all(node, q, timeout=10.0):
    """Publish lift_joint=q until every frame resolves with the carriage at q."""
    end = time.monotonic() + timeout
    missing = list(EXPECTED_FRAMES)
    while time.monotonic() < end:
        msg = JointState(name=['lift_joint'], position=[q])
        msg.header.stamp = node.get_clock().now().to_msg()
        node.joint_pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
        found, missing = {}, []
        for frame in EXPECTED_FRAMES:
            try:
                found[frame] = node.tf_buffer.lookup_transform(ROOT, frame, Time())
            except Exception:  # noqa: BLE001 - any tf2 lookup failure means missing
                missing.append(frame)
        if not missing:
            base = found['lift_column_base_link'].transform.translation.z
            carriage = found['lift_carriage_link'].transform.translation.z
            if abs(carriage - base - CONFIG['column'][
                    'dold_v4_350']['retracted_length'] - q) < TOL:
                return found
    raise AssertionError(f'frames missing from TF at lift {q}: {missing}')


@pytest.mark.parametrize('combo', BUILDABLE, ids=lambda c: '-'.join(c.values()))
def test_all_frames_in_tf(node, combo):
    assert combo['column'] == 'dold_v4_350'
    node.tf_buffer.clear()
    proc = _start_rsp(expand(**combo))
    try:
        stroke = CONFIG['column'][combo['column']]['stroke']
        for q, shoulder in ((0.0, 1.000), (stroke, 1.350)):
            tfs = _lookup_all(node, q)
            for side in ('left', 'right'):
                z = tfs[f'openarm_{side}_base_link'].transform.translation.z
                assert abs(z - shoulder) < TOL, (side, q, z)
            cam = tfs['head_camera_link'].transform.translation.z
            assert cam <= CONFIG['geometry']['max_assembled_height'] + TOL
    finally:
        _stop(proc)
