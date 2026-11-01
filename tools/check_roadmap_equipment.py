#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Check D8's roadmap reference against the owning implementation datums.

Read-only: supply the pinned openamrobot-docs checkout's model/model.js. Numeric
values remain in datums.yaml; this mapping contains keys, never a second design.
"""
import argparse
import json
from pathlib import Path
import re
import yaml

CHEST = {
    'second_estop_below_static_top': 'second_estop.below_top',
    'second_estop_panel_offset': 'second_estop.panel_ahead',
    'head_light_size': 'drawn.head_light.size', 'head_light_z': 'drawn.head_light.z',
    'getac_tablet_size': 'drawn.getac_tablet.size', 'getac_tablet_z': 'drawn.getac_tablet.z',
    'head_light_window_width': 'drawn.head_light.window.0',
    'head_light_window_height': 'drawn.head_light.window.1',
}
LIFT = {
    'lift_sensor_z_l1000': 'z.0', 'lift_sensor_z_l1175': 'z.1', 'lift_sensor_z_l1350': 'z.2',
    'lift_sensor_axis_offset': 'axis_off', 'lift_sensor_diameter': 'sensor.d',
    'lift_sensor_length': 'sensor.len', 'lift_sensor_face_x': 'face_x', 'lift_sensor_gap': 'gap',
    'lift_flag_size': 'flag', 'lift_flag_y_offset': 'flag_off', 'lift_flag_below_plate': 'flag_below_plate',
    'lift_flag_rod_width': 'rod.w', 'lift_flag_rod_length': 'rod.len', 'lift_flag_rod_offset': 'rod.off',
    'lift_sensor_bracket_leg': 'bracket.leg', 'lift_sensor_bracket_thickness': 'bracket.t',
    'lift_sensor_bracket_at': 'bracket.at', 'lift_sensor_bracket_height': 'bracket_h',
}


def block(text, name):
    match = re.search(r'/\*'+name+r'-JSON\*/(.*?)/\*'+name+r'-JSON-END\*/', text, re.S)
    if not match:
        raise ValueError(f'Missing {name}-JSON block')
    return json.loads(match.group(1))


def mismatches(text, datums):
    errors = []
    for name, mapping in [('CHEST', CHEST), ('LIFT', LIFT)]:
        source = block(text, name)
        for datum, path in mapping.items():
            value = source
            for key in path.split('.'):
                value = value[int(key)] if isinstance(value, list) else value[key]
            if value != datums[datum]['value']:
                errors.append(f'{datum}: implementation {datums[datum]["value"]}, roadmap {value}')
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--datums', type=Path, default=Path(__file__).resolve().parents[1] /
                        'ros2/src/openamrobot_upperbody_description/config/datums.yaml')
    args = parser.parse_args()
    errors = mismatches(args.model.read_text(), yaml.safe_load(args.datums.read_text())['datums'])
    print(json.dumps({'checked': len(CHEST)+len(LIFT), 'mismatches': errors,
                      'classification': 'reference consistency, not physical acceptance'}, indent=2))
    return bool(errors)


if __name__ == '__main__':
    raise SystemExit(main())
