# SPDX-License-Identifier: MIT
"""D8 fixed/moving ownership, source dimensions and sensor/flag geometry."""
import numpy as np
import pytest
from conftest import expand, load_datums
from urdf_utils import zero_pose_poses, joint_origin


@pytest.mark.parametrize('q,stop', [(0., 'l1000'), (.175, 'l1175'), (.35, 'l1350')])
def test_flag_meets_each_fixed_sensor_at_its_calibrated_stop(q, stop):
    root = expand()
    poses = zero_pose_poses(root, {'lift_joint': q})
    dt = load_datums()['datums']
    sensor = poses[f'lift_stop_{stop}_link'][:3, 3]
    flag = poses['lift_flag_link'][:3, 3]
    sx, sy, _ = dt['lift_flag_size']['value']
    assert sensor[2] == pytest.approx(flag[2], abs=1e-10)
    assert sensor[0] - (flag[0]+sx/2) == pytest.approx(dt['lift_sensor_gap']['value'], abs=1e-10)
    assert flag[1]-sy/2 < sensor[1] < flag[1]+sy/2


def test_static_equipment_stays_fixed_and_moving_equipment_follows_the_lift():
    root = expand()
    p0, p1 = zero_pose_poses(root), zero_pose_poses(root, {'lift_joint': .35})
    fixed = ['second_estop_link'] + [f'lift_stop_{s}_link' for s in ['l1000','l1175','l1350']]
    moving = ['getac_tablet_link', 'head_light_link', 'head_camera_housing_link', 'lift_flag_link',
              'lift_flag_rod_link', 'lift_rod_corner_top_link', 'lift_rod_corner_side_link']
    for name in fixed:
        assert np.allclose(p1[name], p0[name], atol=1e-12)
    for name in moving:
        assert p1[name][:3,3] - p0[name][:3,3] == pytest.approx([0,0,.35], abs=1e-12)
    lift = root.find("joint[@name='lift_joint']/limit")
    assert float(lift.get('effort')) == 0 and float(lift.get('velocity')) == 0


@pytest.mark.parametrize('stop', ['l1000', 'l1175', 'l1350'])
def test_sensor_body_passes_through_the_bracket_aperture(stop):
    root = expand()
    poses = zero_pose_poses(root)
    sensor, bracket = f'lift_stop_{stop}_link', f'lift_stop_{stop}_link_bracket_face'
    dt = load_datums()['datums']
    radius = dt['lift_sensor_diameter']['value']/2
    # The bounding square around the cylindrical sensor must clear every solid
    # rectangle of the bracket face; this catches the previous solid slab.
    collisions = root.findall(f"link[@name='{bracket}']/collision")
    assert len(collisions) == 4
    axis = poses[sensor][:3,3]
    for collision in collisions:
        center = (poses[bracket] @ joint_origin(collision))[:3,3]
        size = np.array([float(v) for v in collision.find('geometry/box').get('size').split()])
        assert abs(axis[1]-center[1]) > size[1]/2+radius or abs(axis[2]-center[2]) > size[2]/2+radius
