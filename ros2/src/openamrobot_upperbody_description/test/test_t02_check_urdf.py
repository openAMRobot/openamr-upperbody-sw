"""T02: `check_urdf` exits 0 for every T01 expansion (the `openarm` gazebo and real expansions at the baseline
included), plus the planar-flag expansions (extra, labelled). The matrix and the runner are scripts/check_urdf_matrix.py,
which verify_a6.sh runs too; here the same runner is checked, and shown to be able to fail.

check_urdf missing is a FAIL, not a skip: a check that cannot run proves nothing, and SKIP would let T02 vanish from
the evidence on a machine without it.
"""
import shutil

import check_urdf_matrix as t02
import pytest

from conftest import BASELINE, SHARE, UPPERBODY, expand_xml
import evidence_common as ev


def test_t02_check_urdf_is_installed():
    assert shutil.which("check_urdf"), "check_urdf not on PATH: T02 cannot run (ros-<distro>-urdfdom / liburdfdom-tools)"


def test_t02_every_t01_expansion_is_accepted():
    rows = t02.run_matrix(SHARE)
    required = [r for r in rows if not r["extra"]]
    assert len(required) == 14                                           # one factor at a time around the baseline, see T01
    base = "/".join(BASELINE.values())
    assert {r["id"] for r in required} >= {f"{base}/openarm/gazebo", f"{base}/openarm/real"}
    assert {r["args"]["base_plate"] for r in required} == set(UPPERBODY["configuration_axes"]["base_plate"])
    assert all("mast_id" not in r["args"] for r in rows)
    rejected = [(r["id"], r["returncode"], r["summary"]) for r in rows if r["returncode"] != 0]
    assert not rejected, rejected


def test_t02_the_planar_expansions_are_checked_and_accepted():
    # W02 says check_urdf accepts the planar joint; this is where that is run
    extras = [r for r in t02.run_matrix(SHARE) if r["extra"] and r["args"]["enable_planar_base"] == "true"]
    assert len(extras) == 3 and all(r["returncode"] == 0 for r in extras)
    assert 'type="planar"' in expand_xml(enable_planar_base="true")


def test_t02_gazebo_and_real_are_also_checked_at_the_other_base_plate():
    rows = {r["id"]: r for r in t02.run_matrix(SHARE)}
    other = next(p for p in UPPERBODY["configuration_axes"]["base_plate"] if p != BASELINE["base_plate"])
    for profile in ("gazebo", "real"):
        label = ev.row_label({**BASELINE, "base_plate": other, "arm_package": "openarm", "hardware_profile": profile})
        row = rows[label]
        assert row["extra"] and row["returncode"] == 0 and row["args"]["hardware_profile"] == profile


@pytest.mark.parametrize("urdf", [
    "<robot name='r'><link name='a'/>",                                                                  # not XML
    "<robot name='r'><link name='a'/><link name='b'/></robot>",                                          # two roots
    "<robot name='r'><link name='a'/><link name='b'/>"
    "<joint name='j' type='fixed'><parent link='a'/><child link='nope'/></joint></robot>",               # dangling child
    "<robot name='r'><link name='a'/><link name='a'/></robot>",                                          # duplicate link
])
def test_t02_runner_reports_a_rejected_urdf_as_non_zero(urdf):
    rc, output = t02.run_check_urdf(urdf)
    assert rc not in (0, None), output


def test_t02_runner_accepts_a_good_urdf():
    assert t02.run_check_urdf("<robot name='r'><link name='a'/></robot>")[0] == 0


def test_t02_a_missing_tool_fails_the_command_and_its_sidecar(monkeypatch, tmp_path):
    monkeypatch.setenv(t02.ev.EVIDENCE_ENV, str(tmp_path))
    monkeypatch.setattr(t02.shutil, "which", lambda _name: None)
    rc, output = t02.run_check_urdf("<robot name='r'><link name='a'/></robot>")
    assert rc is None and "check_urdf not found" in output
    assert t02.main() == 1                                               # nothing could be checked, so nothing passed
    sidecar = t02.ev.read_sidecars(str(tmp_path))["T02"]
    assert sidecar["result"] == "FAIL" and len(sidecar["details"]["failed"]) == 19         # 14 + 3 planar + 2 profile extras
