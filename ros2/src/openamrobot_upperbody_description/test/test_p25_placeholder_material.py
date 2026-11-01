# SPDX-License-Identifier: MIT
"""P25: reference finishes in RViz and the actual Harmonic URDF-to-SDF conversion.

Neutral colours no longer mark provisional geometry; provenance does. Multi-finish
links use named visuals and native SDFormat conversion (RGB * 1.25, clamped).
Structural unnamed visuals retain exact explicit ambient/diffuse overrides.
"""
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest
from conftest import MATRIX, expand, expand_xml

PALETTE = {
    'structural_aluminum': [.63, .67, .71, 1], 'adapter_gray': [.25, .28, .31, 1],
    'equipment_black': [.035, .045, .055, 1], 'equipment_metal': [.45, .48, .52, 1],
    'sensor_dark': [.09, .10, .11, 1], 'estop_red': [.8, .025, .025, 1],
    'flag_steel': [.2, .22, .25, 1],
}
STRUCTURAL = {'lift_column_base_link':'structural_aluminum', 'lift_carriage_link':'structural_aluminum',
              'arm_profile_link':'structural_aluminum', 'adapter_plate_link':'adapter_gray'}
EQUIPMENT = {'placeholder_getac_tablet':'equipment_black', 'placeholder_head_light':'equipment_metal',
             'placeholder_second_estop_contact_block':'equipment_black', 'placeholder_second_estop_collar':'equipment_metal',
             'placeholder_second_estop_head':'estop_red', 'lift_stop_sensor_body':'sensor_dark',
             'placeholder_lift_flag':'flag_steel', 'placeholder_head_camera_body':'equipment_black',
             'placeholder_head_camera_bracket_foot':'structural_aluminum',
             'placeholder_head_camera_bracket_post':'structural_aluminum'}

WRIST = {side+'_wrist_camera_link': 'equipment_black' for side in ['left','right']}

def finish(visual):
    return EQUIPMENT.get(visual.get('name'), 'structural_aluminum')


def floats(text):
    return [float(v) for v in text.split()]


@pytest.mark.parametrize('base_plate,arm_package', MATRIX, ids=[f'{p}-{a}' for p,a in MATRIX])
def test_urdf_material_and_gazebo_block(base_plate, arm_package):
    root = expand(base_plate=base_plate, arm_package=arm_package)
    for link, material in STRUCTURAL.items():
        visuals = root.findall(f"link[@name='{link}']/visual")
        assert visuals
        for visual in visuals:
            assert visual.find('material').get('name') == material
            assert floats(visual.find('material/color').get('rgba')) == pytest.approx(PALETTE[material])
            assert visual.get('name') is None
        for tag in ['ambient','diffuse']:
            assert floats(root.findtext(f"gazebo[@reference='{link}']/visual/material/{tag}")) == pytest.approx(PALETTE[material])
    if arm_package == 'openarm':
        for link, material in WRIST.items():
            visual = root.find(f"link[@name='{link}']/visual")
            assert visual.find('material').get('name') == material
            assert floats(visual.find('material/color').get('rgba')) == pytest.approx(PALETTE[material])
    seen = set()
    for visual in root.findall('link/visual'):
        name = visual.get('name','')
        if name in EQUIPMENT or name.startswith(('placeholder_lift_sensor_bracket','placeholder_lift_flag_rod','placeholder_lift_rod_corner')):
            material = finish(visual); seen.add(name)
            assert visual.find('material').get('name') == material
            assert floats(visual.find('material/color').get('rgba')) == pytest.approx(PALETTE[material])
    assert set(EQUIPMENT) <= seen


def test_placeholder_colour_is_replaced_without_recolouring_openarm():
    root = expand()
    assert not root.findall(".//material[@name='placeholder_geometry']")
    for link in root.findall('link'):
        if link.get('name','').startswith('openarm_'):
            assert not any(v.find('material') is not None and v.find('material').get('name') in PALETTE for v in link.findall('visual'))


@pytest.mark.skipif(shutil.which('gz') is None, reason='gz unavailable; conversion tracked by upperbody PR #27')
@pytest.mark.parametrize('base_plate', ['bp000','bp050'])
def test_sdf_carries_the_intended_individual_finishes(base_plate, tmp_path):
    urdf = tmp_path/'robot.urdf';urdf.write_text(expand_xml(base_plate=base_plate))
    sdf = ET.fromstring(subprocess.run(['gz','sdf','-p',str(urdf)],capture_output=True,text=True,check=True,timeout=120).stdout)
    seen = set()
    for visual in sdf.findall('.//visual'):
        name = visual.get('name','')
        targets = [(link,mat,False) for link,mat in STRUCTURAL.items()] + [(key,mat,True) for key,mat in (EQUIPMENT | WRIST).items()]
        for key, mat, native in targets:
            if re.search(rf'(^|__){re.escape(key)}_visual(_\d+)?$',name):
                seen.add(key);expected=PALETTE[mat]
                if native:expected=[min(1.,v*1.25) for v in expected[:3]]+[expected[3]]
                for tag in ['ambient','diffuse']:
                    assert floats(visual.findtext('material/'+tag)) == pytest.approx(expected,abs=1e-6), name
    assert seen == set(STRUCTURAL)|set(EQUIPMENT)|set(WRIST)
