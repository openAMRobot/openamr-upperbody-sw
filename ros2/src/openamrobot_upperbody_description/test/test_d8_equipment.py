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
    sensor = poses[f'lift_stop_sensor_{stop}_link'][:3, 3]
    flag = poses['lift_flag_link'][:3, 3]
    sx, sy, _ = dt['lift_flag_size']['value']
    assert sensor[2] == pytest.approx(flag[2], abs=1e-10)
    assert sensor[0] - (flag[0]+sx/2) == pytest.approx(dt['lift_flag_gap']['value'], abs=1e-10)
    assert flag[1]-sy/2 < sensor[1] < flag[1]+sy/2


def test_static_equipment_stays_fixed_and_moving_equipment_follows_the_lift():
    root = expand()
    p0, p1 = zero_pose_poses(root), zero_pose_poses(root, {'lift_joint': .35})
    fixed = ['second_estop_link'] + [f'lift_stop_sensor_{s}_link' for s in ['l1000','l1175','l1350']]
    moving = ['getac_tablet_link', 'head_light_link', 'head_camera_link', 'lift_flag_link',
              'lift_flag_rod_link']
    for name in fixed:
        assert np.allclose(p1[name], p0[name], atol=1e-12)
    for name in moving:
        assert p1[name][:3,3] - p0[name][:3,3] == pytest.approx([0,0,.35], abs=1e-12)
    rod = root.find("link[@name='lift_flag_rod_link']")
    assert {c.get('name') for c in rod.findall('collision')} == {
        'placeholder_lift_flag_rod', 'placeholder_lift_rod_corner_side', 'placeholder_lift_rod_corner_top'}
    lift = root.find("joint[@name='lift_joint']/limit")
    assert float(lift.get('effort')) == 0 and float(lift.get('velocity')) == 0


@pytest.mark.parametrize('stop', ['l1000', 'l1175', 'l1350'])
def test_sensor_body_passes_through_the_bracket_aperture(stop):
    root = expand()
    poses = zero_pose_poses(root)
    sensor = bracket = f'lift_stop_sensor_{stop}_link'
    dt = load_datums()['datums']
    radius = dt['lift_stop_sensor_diameter']['value']/2
    # The bounding square around the cylindrical sensor must clear every solid
    # rectangle of the bracket face; this catches the previous solid slab.
    collisions = [c for c in root.findall(f"link[@name='{bracket}']/collision")
                  if c.get("name", "").startswith("placeholder_lift_sensor_bracket_sensor_leg_")]
    assert len(collisions) == 4
    axis = poses[sensor][:3,3]
    for collision in collisions:
        center = (poses[bracket] @ joint_origin(collision))[:3,3]
        size = np.array([float(v) for v in collision.find('geometry/box').get('size').split()])
        assert abs(axis[1]-center[1]) > size[1]/2+radius or abs(axis[2]-center[2]) > size[2]/2+radius


def test_selected_falcon_housing_and_illumination_area_match_supplier_drawing():
    """FLDL-i150x15-W Rev 1.6 p1/p2; reject the smaller roadmap envelope."""
    root = expand()
    housing = root.find("link[@name='head_light_link']/collision/geometry/box")
    assert [float(v) for v in housing.get('size').split()] == pytest.approx([.020, .158, .0175])
    window = root.find("link[@name='head_light_window_link']/visual/geometry/box")
    assert [float(v) for v in window.get('size').split()][1:] == pytest.approx([.150, .015])
    assert float(root.find("link[@name='head_light_link']/inertial/mass").get('value')) == .110
    poses = zero_pose_poses(root)
    tablet_bottom = poses['getac_tablet_link'][2, 3]-.1498/2
    light_top = poses['head_light_link'][2, 3]+.0175/2
    assert tablet_bottom-light_top == pytest.approx(.030)


@pytest.mark.parametrize('stop', ['l1000', 'l1175', 'l1350'])
def test_selected_sick_bracket_envelope_and_bore_location_match_supplier_drawing(stop):
    """5308447 drawing p2: 30 mm width/foot, 26.5 mm face, 1.5 mm sheet, 15 mm bore datum."""
    root = expand()
    poses = zero_pose_poses(root)
    face = f'lift_stop_sensor_{stop}_link'
    mount = root.find(f"link[@name='{face}']/collision[@name='placeholder_lift_sensor_bracket_face_leg']")
    size = [float(v) for v in mount.find('geometry/box').get('size').split()]
    assert size == pytest.approx([.030, .0015, .030])
    lows, highs = [], []
    for collision in root.findall(f"link[@name='{face}']/collision"):
        if not collision.get("name", "").startswith("placeholder_lift_sensor_bracket_sensor_leg_"):
            continue
        center = (poses[face] @ joint_origin(collision))[:3, 3]
        half = np.array([float(v) for v in collision.find('geometry/box').get('size').split()])/2
        lows.append(center-half)
        highs.append(center+half)
    low, high = np.min(lows, axis=0), np.max(highs, axis=0)
    assert high-low == pytest.approx([.0015, .0265-.0015, .030])
    mount_low_y = (poses[face] @ joint_origin(mount))[1, 3]-.0015/2
    assert low[1]-mount_low_y == pytest.approx(.0015)
    assert high[1]-mount_low_y == pytest.approx(.0265)
    sensor = poses[f'lift_stop_sensor_{stop}_link'][:3, 3]
    assert sensor[1]-mount_low_y == pytest.approx(.015)
    assert sensor[2] == pytest.approx((low[2]+high[2])/2)
