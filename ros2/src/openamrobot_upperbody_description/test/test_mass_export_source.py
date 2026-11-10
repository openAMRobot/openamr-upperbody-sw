# SPDX-License-Identifier: MIT
"""The mass export is the reviewed one on openamrobot-docs main (ff8ca57, docs #84), not a branch snapshot, and the platform pin is
left for platform-sw #126 (a TODO, not a guessed commit). test_roadmap_masses.py checks the SHA-256 of the copy against this pin."""
import os

import yaml

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, "config")


def load(name):
    with open(os.path.join(CONFIG, name), encoding="utf-8") as f:
        return f.read()


def test_the_export_is_pinned_to_the_reviewed_docs_main_commit():
    pin = yaml.safe_load(load("roadmap_mass_export.source.yaml"))
    assert pin["repository"] == "openAMRobot/openamrobot-docs"
    assert pin["path"] == "docs/assets/roadmap/model/mass_export.yaml"
    assert pin["commit"] == "ff8ca57f2687c7c0c9b65005da9d24baf5db8e7c"


def test_the_copy_is_the_reviewed_snapshot():
    export = yaml.safe_load(load("roadmap_mass_export.yaml"))
    assert export["snapshot"] == "9 Nov 2026" and "BOM Issue 7.15" in export["about"]


def test_the_platform_pin_is_the_platform_sw_126_merge():
    assert "platform-sw #126" in load("mass_provenance.yaml")
    assert yaml.safe_load(load("upstream_pins.yaml"))["pins"]["openamr_platform_sw"] == "713574fd2a5a9e38cde6b07c5b96ac4f380b3402"
