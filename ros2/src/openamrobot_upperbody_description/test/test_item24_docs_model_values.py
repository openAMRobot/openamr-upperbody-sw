# SPDX-License-Identifier: MIT
"""The robot description and the docs model are the same robot: tools/export_docs_model_values.py, run on the expanded URDF, prints the
CHEST-JSON and LIFT-JSON values of openamrobot-docs model.js that the P-03 item 24 datums came from, within 0.5 mm.

The model.js blocks are a verbatim fixture (test/fixtures/docs_model_item24.json, with the docs commit and a hash of each block), because CI
has no checkout of openamrobot-docs. To compare with a live model.js, run the tool with --model-js (no skipped test stands in for it). Keys
the URDF cannot carry are listed by the tool (NOT_EXPORTED) and are the only ones not compared.
"""
import hashlib
import json
import os
import sys
import xml.etree.ElementTree as ET

import pytest

from conftest import FIXTURES, expand_xml, load_datums

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, os.pardir, os.pardir, os.pardir))
sys.path.insert(0, os.path.join(REPO, "tools"))
import export_docs_model_values as exporter  # noqa: E402

TOLERANCE = 0.0005           # 0.5 mm
FIXTURE = os.path.join(FIXTURES, "docs_model_item24.json")


def fixture():
    with open(FIXTURE) as f:
        doc = json.load(f)
    for block, text in doc["blocks"].items():
        assert hashlib.sha256(text.encode()).hexdigest() == doc["sha256"][block], f"{block}: the fixture was edited, not copied"
    return doc


def leaves(value, prefix=""):
    """{dotted key: leaf} of a JSON value (list entries as key[i])."""
    if isinstance(value, dict):
        out = {}
        for key, item in value.items():
            out.update(leaves(item, f"{prefix}.{key}" if prefix else key))
        return out
    if isinstance(value, list) and value and not all(isinstance(v, str) for v in value):
        out = {}
        for i, item in enumerate(value):
            out.update(leaves(item, f"{prefix}[{i}]"))
        return out
    return {prefix: value}


def differences(docs, exported, skipped):
    """Readable mismatches between the docs block and the export; keys under `skipped` are not compared."""
    want, got = leaves(docs), leaves(exported)
    skip = lambda key: any(key == s or key.startswith(s + "[") or key.startswith(s + ".") for s in skipped)  # noqa: E731
    problems = [f"{k}: in the docs model, not exported" for k in sorted(want) if not skip(k) and k not in got]
    problems += [f"{k}: exported, not in the docs model" for k in sorted(got) if k not in want]
    for key in sorted(set(want) & set(got)):
        a, b = want[key], got[key]
        if isinstance(a, (int, float)) and not isinstance(a, bool):
            if not isinstance(b, (int, float)) or abs(a - b) > TOLERANCE:
                problems.append(f"{key}: docs {a}, URDF {b}")
        elif a != b:
            problems.append(f"{key}: docs {a!r}, URDF {b!r}")
    return problems


@pytest.mark.parametrize("base_plate", ["bp000", "bp050"])
def test_item24_the_export_of_the_urdf_equals_the_docs_model_blocks(base_plate):
    exported = exporter.export(ET.fromstring(expand_xml(base_plate=base_plate, arm_package="none")), load_datums()["datums"])
    doc = fixture()
    for block in ("CHEST-JSON", "LIFT-JSON"):
        problems = differences(json.loads(doc["blocks"][block]), exported[block], exporter.NOT_EXPORTED[block])
        assert not problems, f"{block} at {base_plate} (docs {doc['commit'][:7]}):\n" + "\n".join(problems)


def test_item24_the_comparison_can_fail():
    """A value off by more than 0.5 mm, a missing key and a wrong string are each reported; 0.4 mm is not."""
    docs = {"z": [0.12, 0.295], "face": "left", "sensor": {"d": 0.012}}
    assert differences(docs, {"z": [0.12, 0.2954], "face": "left", "sensor": {"d": 0.012}}, []) == []
    assert differences(docs, {"z": [0.12, 0.2956], "face": "left", "sensor": {"d": 0.012}}, []) == ["z[1]: docs 0.295, URDF 0.2956"]
    assert differences(docs, {"z": [0.12, 0.295], "face": "right", "sensor": {"d": 0.012}}, []) == ["face: docs 'left', URDF 'right'"]
    assert differences(docs, {"z": [0.12, 0.295], "face": "left", "sensor": {}}, []) == ["sensor.d: in the docs model, not exported"]
    assert differences(docs, {"z": [0.12, 0.295], "face": "left", "sensor": {}}, ["sensor.d"]) == []


def test_item24_the_tool_skips_only_what_the_urdf_cannot_carry():
    assert exporter.NOT_EXPORTED == {"CHEST-JSON": ["second_estop.glb_node", "second_estop.glb_axis", "drawn.head_light.window"],
                                     "LIFT-JSON": []}
