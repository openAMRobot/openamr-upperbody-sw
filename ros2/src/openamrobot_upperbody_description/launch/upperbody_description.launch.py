# SPDX-License-Identifier: MIT
# Copyright (c) 2026 OpenAMRobot (Botshare LTD)
"""Publish the OpenAMRobot 2.0 upper-body description (option B).

Starts robot_state_publisher only. The lift height is joint state of
lift_joint and must come from /joint_states; this launch does not command or
simulate the lift.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

PACKAGE = 'openamrobot_upperbody_description'


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)
    with open(os.path.join(share, 'config', 'upperbody_configuration.yaml')) as f:
        cfg = yaml.safe_load(f)
    axes, baseline = cfg['configuration_axes'], cfg['baseline']

    args = [
        DeclareLaunchArgument(axis, default_value=baseline[axis], choices=ids,
                              description=f'{axis} configuration ID')
        for axis, ids in axes.items()
    ]
    args.append(DeclareLaunchArgument(
        'column_axis_x_bp000', default_value='0.060',
        description='Column axis x at bp000 relative to base_footprint (m)'))

    xacro_file = os.path.join(share, 'urdf', 'openamrobot_upperbody.urdf.xacro')
    command = ['xacro ', xacro_file]
    for name in list(axes) + ['column_axis_x_bp000']:
        command += [f' {name}:=', LaunchConfiguration(name)]
    description = ParameterValue(Command(command), value_type=str)

    return LaunchDescription(args + [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[{'robot_description': description}]),
    ])
