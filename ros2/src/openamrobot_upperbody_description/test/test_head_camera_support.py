# SPDX-License-Identifier: MIT
"""The provisional visual support connects existing rigid geometry; it is not hardware CAD."""
import itertools
import numpy as np
import pytest
from conftest import expand
from urdf_utils import fixed_fk, joint_origin, zero_pose_poses


def vector(e, key):
    return np.array([float(v) for v in e.get(key).split()])


@pytest.mark.parametrize('base_plate',['bp000','bp050'])
@pytest.mark.parametrize('head_pitch',['hp15','hp20','hp25','hp30','hp35'])
def test_support_touches_profile_and_housing_without_entering_forward_view(base_plate,head_pitch):
    root=expand(base_plate=base_plate,head_pitch=head_pitch)
    mount=root.find("link[@name='head_camera_mount_link']")
    visuals=mount.findall('visual');assert len(visuals)==2
    assert mount.find('collision') is None and mount.find('inertial') is None
    foot,stem=visuals
    ft=fixed_fk(root,'lift_carriage_link','head_camera_mount_link')@joint_origin(foot)
    profile=root.find("link[@name='arm_profile_link']/visual/geometry/box")
    pt=fixed_fk(root,'lift_carriage_link','arm_profile_link')
    size=vector(foot.find('geometry/box'),'size');ps=vector(profile,'size')
    assert ft[2,3]-size[2]/2 == pytest.approx(pt[2,3]+ps[2]/2,abs=1e-12)
    assert np.all(np.abs(ft[:2,3]-pt[:2,3])+size[:2]/2 <= ps[:2]/2+1e-12)
    # Sample the stem cross-section inside the existing tilted housing. This
    # guards an actual visible connection for every supported camera pitch.
    st=joint_origin(stem);ss=vector(stem.find('geometry/box'),'size')
    camera=fixed_fk(root,'head_camera_mount_link','head_camera_housing_link')
    hs=vector(root.find("link[@name='head_camera_housing_link']/visual/geometry/box"),'size')
    points=np.array([[x,0,z,1] for x,z in itertools.product(np.linspace(-ss[0]/2,ss[0]/2,25),np.linspace(-ss[2]/2,ss[2]/2,80))])
    in_housing=(np.linalg.inv(camera)@st@points.T).T[:,:3]
    assert np.any(np.all(np.abs(in_housing)<=hs/2+1e-12,axis=1)), 'support does not touch housing'
    # The support stays outside the ideal camera's forward near-clipped viewing cone.
    lens=fixed_fk(root,'head_camera_mount_link','head_camera_link')
    for v in visuals:
        dims=vector(v.find('geometry/box'),'size')
        corners=np.array([list(p)+[1] for p in itertools.product(*[[-d/2,d/2] for d in dims])])
        local=(np.linalg.inv(lens)@joint_origin(v)@corners.T).T
        in_view=(local[:,0] >= .02) & (np.abs(local[:,1]) <= local[:,0]) & (np.abs(local[:,2]) <= .75*local[:,0])
        assert not np.any(in_view)


@pytest.mark.parametrize('height',[0.,.175,.350])
def test_support_moves_with_lift_and_adds_no_new_frame_or_dynamics(height):
    root=expand();poses=zero_pose_poses(root,{'lift_joint':height});zero=zero_pose_poses(root)
    for name in ['head_camera_mount_link','head_camera_link','head_camera_optical_frame','head_camera_housing_link']:
        np.testing.assert_allclose(poses[name][:3,3]-zero[name][:3,3],[0,0,height],atol=1e-12)
    assert root.find("joint[@name='head_camera_mount_joint']/parent").get('link')=='lift_carriage_link'
    assert root.find("joint[@name='head_camera_pitch_joint']/origin").get('rpy')=='0 0.5235987755982988 0'
