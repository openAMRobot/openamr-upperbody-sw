# SPDX-License-Identifier: MIT
"""Guard the ideal-floor contact model on both supported engines."""
from pathlib import Path
import xml.etree.ElementTree as ET


def test_floor_cannot_add_default_bullet_spinning_friction():
    world = Path(__file__).resolve().parents[1] / 'worlds/empty.sdf'
    root = ET.parse(world).getroot()
    friction = root.find(".//model[@name='ground_plane']/link/collision/surface/friction")
    assert friction is not None, 'Implicit Bullet ground spinning friction is 1, locking wheel yaw'
    assert float(friction.findtext('torsional/coefficient')) == 0
    assert float(friction.findtext('bullet/rolling_friction')) == 0
    for name in ['bullet/friction', 'bullet/friction2', 'ode/mu', 'ode/mu2']:
        assert float(friction.findtext(name)) == 1, 'Retain the original sliding-friction default'
