# SPDX-License-Identifier: MIT
"""simulation/parameter_register.yaml: the register against its authorities, and every simulator against the register.

Gazebo is the expansion text (the URDF Gazebo gets), MuJoCo the compiled generated MJCF, Isaac the importer URDF and the import job.
A value that differs from the register fails, unless the check declares that deviation with its value and an issue; a declared
deviation that no longer holds fails too. The USD is not read (Isaac Sim needs a GPU; check_usd.py is BLOCKED here).
"""
import copy
import json

import numpy as np
import pytest

from openamr_sim import isaac
from openamr_sim import register as reg
from openamr_sim.expand import REPO

REGISTER = reg.load()
ROWS = REGISTER["parameters"]
SIMS = {"gazebo", "mujoco", "isaac"}
CHECKS = [(name, k) for name, row in ROWS.items() for k in range(len(row["checks"]))]
CONFIGS = REGISTER["configurations"]


def tol(row):
    return row.get("tolerance", 1e-9)


@pytest.fixture(scope="module")
def reps(generated, models, tmp_path_factory):
    out = {}
    for name in CONFIGS:
        g = generated[name]
        job_dir = tmp_path_factory.mktemp(f"isaac_{name}")
        job = isaac.build_job(g["expansion"], job_dir)
        out[name] = reg.Representations(g["expansion"].text, models[name], g["report"], job, (job_dir / job["urdf"]).read_text())
    return out


# ------------------------------------------------------------------------------------------------ the register itself
@pytest.mark.parametrize("name", sorted(ROWS))
def test_row_has_every_field(name):
    row = ROWS[name]
    missing = [f for f in reg.REQUIRED if f not in row]
    assert not missing, f"{name}: missing {missing}"
    assert row["verification"] in REGISTER["verification_levels"], f"{name}: verification {row['verification']}"
    owner = row["owner"]
    assert owner in REGISTER["owners"] or owner.startswith(("Team doc s7", "ledger: Team doc s7", "to be assigned by Alex")), \
        f"{name}: owner {owner!r} is neither a CODEOWNERS key nor a Team doc section"
    assert row["checks"] or row.get("not_carried"), f"{name}: no check and no not_carried reason"
    for c in row["checks"]:
        assert c["sim"] in SIMS, f"{name}: simulator {c['sim']}"
        assert c["read"].split(".", 1)[0] in {"urdf", "isaac_urdf", "gazebo", "mujoco", "isaac"}, f"{name}: reader {c['read']}"
        if "deviation" in c:
            d = c["deviation"]
            assert d.get("issue") in REGISTER["issues"], f"{name}: deviation without a tracking issue"
            assert not reg.same(d["value"], reg.nominal(row, c), tol(row)), f"{name}: deviation equals the register value"


def _authority_value(auth, platform_sw):
    import yaml
    root = REPO if auth["repo"] == "upperbody_sw" else platform_sw
    data = yaml.safe_load((root / auth["file"]).read_text())
    if "pose_keys" in auth:
        node = reg.lookup(data, auth["pose_keys"])
        return {"xyz": node["xyz"], "rpy": node["rpy"]}
    value = reg.lookup(data, auth["keys"])
    if "order" in auth:
        value = [value[k] for k in auth["order"]]
    return value


FILE_AUTHORITIES = sorted(n for n, r in ROWS.items() if "repo" in r["authority"])


@pytest.mark.parametrize("name", FILE_AUTHORITIES)
def test_register_value_equals_its_authority(name, platform_sw):
    row = ROWS[name]
    want = _authority_value(row["authority"], platform_sw)
    got = row["value"]
    if isinstance(got, dict):
        assert reg.same(reg.pose_matrix(got), want, 1e-12), f"{name}: register {got}, authority {want}"
    else:
        assert reg.same(got, want, 1e-12), f"{name}: register {got}, authority {row['authority']['file']} {want}"
    if "also_equal" in row:
        other = _authority_value(row["also_equal"], platform_sw)
        assert reg.same(other, want, 1e-12), f"{name}: hand copy {row['also_equal']['file']} {other} differs from {want}"


def test_derived_contact_height_follows_its_inputs():
    expect = ROWS["base_link_height_above_floor"]["value"] - ROWS["drive_wheel_radius_effective"]["value"]
    assert abs(ROWS["drive_wheel_contact_height"]["value"] - expect) < 1e-12


def test_every_row_without_a_file_authority_says_so():
    for name, row in ROWS.items():
        a = row["authority"]
        assert "repo" in a or "derived" in a or a.get("machine_checked") is False, f"{name}: authority {a} is not checkable"


# ------------------------------------------------------------------------------------------------ simulators against the register
@pytest.mark.parametrize("config", CONFIGS)
@pytest.mark.parametrize("name,k", CHECKS, ids=[f"{n}-{k}" for n, k in CHECKS])
def test_simulator_carries_the_register_value(reps, config, name, k):
    row, c = ROWS[name], ROWS[name]["checks"][k]
    actual = reps[config].read(c)
    want = reg.expected(row, c)
    where = f"{name} [{c['sim']} {c['read']} {', '.join(f'{x}={c[x]}' for x in c if x not in ('sim', 'read', 'deviation', 'note'))}]"
    if "deviation" in c and reg.same(actual, reg.nominal(row, c), tol(row)):
        pytest.fail(f"{where}: carries the register value {reg.nominal(row, c)} now; remove the declared deviation")
    assert reg.same(actual, want, tol(row)), (
        f"{where}: {reg.describe(actual)}, register {reg.nominal(row, c)}"
        + (f", declared deviation {c['deviation']['value']}" if "deviation" in c else ""))


def test_findings_summary(reps, evidence):
    """Every declared deviation, with the value read now; written to SIM_EVIDENCE_DIR as parameter_register_findings.json."""
    rows = []
    for name, k in CHECKS:
        row, c = ROWS[name], ROWS[name]["checks"][k]
        if "deviation" not in c:
            continue
        actual = reps["bp000"].read(c)
        rows.append({"parameter": name, "simulator": c["sim"], "read": c["read"], "register": row["value"],
                     "simulated": reg.describe(actual), "verification": row["verification"], "note": c["deviation"]["note"],
                     "issue": REGISTER["issues"][c["deviation"]["issue"]]})
    evidence("parameter_register_findings", {"configuration": "bp000", "deviations": rows})
    assert rows, "the register declares no deviation; then this summary has nothing to report"
    json.dumps(rows, default=float)


# ------------------------------------------------------------------------------------------------ the checks can fail
def _changed(rep, **kw):
    other = copy.copy(rep)
    for k, v in kw.items():
        setattr(other, k, v)
    return other


def test_mutation_mujoco_wheel_radius(reps):
    rep = reps["bp000"]
    m = copy.deepcopy(rep.m)
    gid = rep.mj.mj_name2id(m, rep.mj.mjtObj.mjOBJ_GEOM, "left_wheel_link__collision_0")
    m.geom_size[gid][0] = 0.098
    other = _changed(rep, m=m)
    row = ROWS["drive_wheel_radius_effective"]
    c = next(c for c in row["checks"] if c["read"] == "mujoco.geom_size")
    assert not reg.same(other.read(c), reg.expected(row, c), tol(row))


def test_mutation_gazebo_track(reps, generated):
    rep = reps["bp000"]
    import re
    original = generated["bp000"]["expansion"].text
    text = re.sub(r'(<joint name="left_wheel_joint".*?<origin rpy="0 0 0" xyz=")0 0\.200 0"', r'\g<1>0 0.230 0"', original,
                  count=1, flags=re.S)
    assert text != original
    other = _changed(rep, urdf=reg.URDF(text))
    row = ROWS["drive_track"]
    c = row["checks"][0]
    assert not reg.same(other.read(c), row["value"], tol(row))


def test_mutation_mujoco_camera_body(reps):
    rep = reps["bp000"]
    m = copy.deepcopy(rep.m)
    k = rep.mj.mj_name2id(m, rep.mj.mjtObj.mjOBJ_CAMERA, "base_camera")
    m.cam_bodyid[k] = rep.mj.mj_name2id(m, rep.mj.mjtObj.mjOBJ_BODY, "base_camera_color_optical_frame")
    other = _changed(rep, m=m)
    row = ROWS["base_camera_pose"]
    c = next(c for c in row["checks"] if c["read"] == "mujoco.camera")
    assert other.read(c) == reg.nominal(row, c)  # the fix would now trip the "remove the declared deviation" failure


def test_mutation_register_value(platform_sw):
    row = copy.deepcopy(ROWS["lidar_pose"])
    row["value"]["xyz"][0] = 0.321
    want = _authority_value(row["authority"], platform_sw)
    assert not reg.same(reg.pose_matrix(row["value"]), want, 1e-12)


def test_pose_comparison_detects_a_rotation():
    T = reg.pose_matrix({"xyz": [0.33, 0.0, 0.124], "rpy": [0.0, 0.0, np.pi]})
    assert reg.same(T, {"xyz": [0.33, 0.0, 0.124], "rpy": [0.0, 0.0, np.pi]}, 1e-9)
    assert not reg.same(T, {"xyz": [0.33, 0.0, 0.124], "rpy": [0.0, 0.0, 0.0]}, 1e-9)
