# SPDX-License-Identifier: MIT
"""Proposed ideal camera producers use the existing optical frame contract."""
from pathlib import Path
import numpy as np
import pytest
import yaml
from conftest import expand
from urdf_utils import zero_pose_poses
PROFILE=Path(__file__).resolve().parents[1]/'config/simulation_cameras.yaml'
@pytest.mark.parametrize('name',['head_camera','left_wrist_camera','right_wrist_camera'])
def test_ideal_camera_origin_intrinsics_and_frame(name):
    cfg=yaml.safe_load(PROFILE.read_text()); spec=cfg['cameras'][name];p=cfg['profile']
    root=expand(hardware_profile='gazebo');poses=zero_pose_poses(root)
    sensor=root.find(f"gazebo[@reference='{spec['link']}']/sensor[@name='{name}_ideal']")
    assert sensor is not None
    assert sensor.findtext('gz_frame_id')==spec['optical_frame']
    assert sensor.findtext('topic')==spec['image']
    assert sensor.findtext('camera/camera_info_topic')==spec['camera_info']
    assert float(sensor.findtext('update_rate'))==p['rate_hz']
    assert int(sensor.findtext('camera/image/width'))==p['width']
    assert int(sensor.findtext('camera/image/height'))==p['height']
    assert float(sensor.findtext('camera/horizontal_fov'))==p['horizontal_fov_rad']
    assert np.allclose(poses[spec['link']][:3,3], poses[spec['optical_frame']][:3,3])
    # Gazebo rays use link +X; the ROS optical axis must be the same world vector.
    assert np.allclose(poses[spec['link']][:3,0], poses[spec['optical_frame']][:3,2])

def test_ideal_camera_profile_never_loads_for_mock_or_real_hardware():
    for profile in ['mock','real']:
        root=expand(hardware_profile=profile)
        assert not [s for s in root.findall('.//sensor') if s.get('name','').endswith('_ideal')]
