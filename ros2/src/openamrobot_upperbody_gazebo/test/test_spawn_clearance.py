# SPDX-License-Identifier: MIT
"""The full model starts just clear of its contact plane, without a large drop."""
import importlib.util
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
import xacro
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / 'openamrobot_upperbody_description/test'))
from urdf_utils import joint_origin, zero_pose_poses, pose_to_matrix  # noqa: E402


@pytest.fixture
def model_and_launch():
    spec = importlib.util.spec_from_file_location('sim_spawn', HERE.parent / 'launch/sim.launch.py')
    sim = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sim)
    description = Path(get_package_share_directory('openamrobot_upperbody_description'))
    robot = ET.fromstring(xacro.process_file(str(description / 'urdf/combined_robot.xacro')).toxml())
    return robot, sim


def spawn_arguments(sim):
    """Inspect the actual create command, including the old flat-spawn regression."""
    context = LaunchContext()
    description = sim.generate_launch_description()
    nodes = [e for e in description.entities if isinstance(e, Node)]
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    for entity in description.entities:
        if isinstance(entity, OpaqueFunction):
            nodes.extend(e for e in entity.execute(context) if isinstance(e, Node))
    node, = [n for n in nodes if n.node_executable == 'create']
    return [''.join(s.perform(context) for s in word) for word in node.cmd[1:]]


def clearances(robot, z, pitch):
    poses = zero_pose_poses(robot)
    world = pose_to_matrix((0, 0, z), (0, pitch, 0))
    result = {}
    for name in ['left_wheel_link', 'right_wheel_link', 'fl_wheel_link', 'fr_wheel_link', 'bl_wheel_link', 'br_wheel_link']:
        collision = robot.find(f"link[@name='{name}']/collision")
        pose = world @ poses[name] @ joint_origin(collision)
        cylinder = collision.find('geometry/cylinder')
        if cylinder is not None:
            radius, half_length = float(cylinder.get('radius')), float(cylinder.get('length')) / 2
            axis_z = pose[2, 2]
            extent = radius * np.sqrt(1 - axis_z**2) + half_length * abs(axis_z)
        else:
            extent = float(collision.find('geometry/sphere').get('radius'))
        result[name] = pose[2, 3] - extent
    return result


def test_spawn_is_clear_and_within_five_mm_of_the_ground(model_and_launch):
    robot, sim = model_and_launch
    args = spawn_arguments(sim)
    z = float(args[args.index('-z') + 1])
    pitch = float(args[args.index('-P') + 1]) if '-P' in args else 0.
    gaps = clearances(robot, z, pitch)
    for name in ['left_wheel_link', 'right_wheel_link']:
        # Proposed spawn-placement bound; not a physical performance tolerance.
        assert 0 < gaps[name] <= 0.005, (name, gaps[name])
    assert min(gaps.values()) >= 0, gaps


def test_actual_spawn_has_four_near_contact_supports(model_and_launch):
    robot, sim = model_and_launch
    args = spawn_arguments(sim)
    z = float(args[args.index('-z') + 1])
    pitch = float(args[args.index('-P') + 1]) if '-P' in args else 0.
    gaps = clearances(robot, z, pitch)
    # Numerical startup regression, not a physical contact/load tolerance.
    assert sum(0 < gap <= 2e-6 for gap in gaps.values()) == 4, gaps


def test_spawn_tracks_consumed_contact_geometry(model_and_launch):
    robot, sim = model_and_launch
    z1, pitch1 = sim.ground_support_pose(ET.tostring(robot))
    # Change only the consumed wheel collision radius. No cached CAD constants
    # may leave the robot at its old height/angle or intersect the floor.
    for name in ['left_wheel_link', 'right_wheel_link']:
        robot.find(f"link[@name='{name}']/collision/geometry/cylinder").set('radius', '0.101')
    z2, pitch2 = sim.ground_support_pose(ET.tostring(robot))
    assert z2 > z1 and pitch2 > pitch1
    gaps = clearances(robot, z2, pitch2)
    assert min(gaps.values()) >= 0 and sum(g <= 2e-6 for g in gaps.values()) == 4


def test_unsupported_contact_layout_fails_explicitly(model_and_launch):
    robot, sim = model_and_launch
    robot.find("joint[@name='fr_wheel_joint']/origin").set('xyz', '0 0 0.02')
    with pytest.raises(ValueError, match='No supported wheel/castor plane'):
        sim.ground_support_pose(ET.tostring(robot))
