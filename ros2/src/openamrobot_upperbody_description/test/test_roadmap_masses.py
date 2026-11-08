# SPDX-License-Identifier: MIT
"""Carriage masses from the roadmap model (owner decision 8 Nov 2026, P-03 rev20.5 item 29: the roadmap model is the mass reference).

config/roadmap_mass_export.yaml is a verbatim copy of openamrobot-docs docs/assets/roadmap/model/mass_export.yaml. The parts that
ride the lift carriage (adapter plate, arm profile, tablet mount, second E-stop, flag rod and flag) are lumped on lift_carriage_link,
each BOM line sits on exactly one link, and no carriage part is left on the fixed base."""
import os

import pytest

from conftest import SHARE, expand, load_yaml

EXPORT = load_yaml(os.path.join(SHARE, "config", "roadmap_mass_export.yaml"))
CARRIAGE = EXPORT["groups"]["lift_carriage"]["links"]["lift_carriage_link"]
CARRIAGE_BOM = {"MEC-003", "MEC-004", "MEC-012", "LFT-004", "LFT-005", "LFT-006"}   # plus the second E-stop of SAF-001


def items_by_link():
    out = {}
    for group in EXPORT["groups"].values():
        for link, entry in group["links"].items():
            out[link] = entry["items"]
    return out


def test_lift_carriage_link_carries_the_roadmap_carriage_mass():
    root = expand(arm_package="openarm")
    link = [e for e in root.findall("link") if e.get("name") == "lift_carriage_link"][0]
    inertial = link.find("inertial")
    assert inertial is not None, "lift_carriage_link has no inertial: the carriage parts would be massless"
    assert float(inertial.find("mass").get("value")) == pytest.approx(CARRIAGE["mass_kg"], abs=1e-6)
    xyz = [float(v) for v in inertial.find("origin").get("xyz").split()]
    assert xyz == pytest.approx(CARRIAGE["com_m"], abs=1e-6)
    inertia = inertial.find("inertia")
    assert [float(inertia.get(k)) for k in ("ixx", "iyy", "izz")] == pytest.approx(
        [CARRIAGE["inertia_kgm2"][k] for k in ("ixx", "iyy", "izz")], abs=1e-9)


def test_every_carriage_part_rides_the_carriage_and_nothing_is_counted_twice():
    by_link = items_by_link()
    seen = {}
    for link, items in by_link.items():
        for it in items:
            key = (it["bom"], it["component"])
            assert key not in seen, f"{key} is on {seen.get(key)} and on {link}"
            seen[key] = link
    for (bom, comp), link in seen.items():
        if bom in CARRIAGE_BOM or comp == "second_estop":
            assert link == "lift_carriage_link", f"{bom} {comp} rides the carriage but sits on {link}"
    assert not any(it["bom"] == "ARM-003" for it in by_link["base_link"]), "the head camera is on head_camera_link, not the base"
