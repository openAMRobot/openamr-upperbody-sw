# SPDX-License-Identifier: MIT
"""The Isaac Sim run kit (simulation/isaac/), checked without a GPU: the script's pins, the job's benchmark values, the result
collector's verdicts, and that the Isaac-side runner parses. The Isaac run itself stays BLOCKED until a GPU run's evidence exists."""
import ast
import json
import pathlib
import shutil
import subprocess
import sys

import pytest
import yaml

from openamr_sim import benchmark as bm
from openamr_sim import expand as ex
from openamr_sim import isaac

SIM = pathlib.Path(__file__).resolve().parents[1]
KIT = SIM / "isaac"
sys.path.insert(0, str(KIT))
import collect_result  # noqa: E402


def test_run_script_parses_and_uses_the_pinned_image_and_xacro_build():
    subprocess.run(["bash", "-n", str(KIT / "run_on_gpu.sh")], check=True)
    out = subprocess.run(["bash", str(KIT / "run_on_gpu.sh"), "--dry-run"], cwd=SIM.parent, capture_output=True, text=True, check=True).stdout
    pins = yaml.safe_load((SIM / "sim_pins.yaml").read_text())
    lock_build = {e["inputs"]["xacro_package_version"] for e in ex.load_lock()["entries"]}.pop()
    assert f"isaac image     {pins['isaac_sim']['container_image']}" in out
    assert f"(xacro {lock_build})" in out
    assert f"platform-sw     {ex.pins()['openamr_platform_sw']}" in out
    assert f"openarm_desc    {ex.pins()['openarm_description']}" in out


def test_isaac_runner_parses_and_imports_isaac_only_inside_main():
    tree = ast.parse((KIT / "isaac_scenarios.py").read_text())
    top = [n for n in tree.body if isinstance(n, (ast.Import, ast.ImportFrom))]
    names = {a.name for n in top for a in n.names} | {n.module for n in top if isinstance(n, ast.ImportFrom)}
    assert not any(str(m).startswith(("isaacsim", "omni", "pxr")) for m in names), names


def test_job_carries_the_benchmark_values_of_the_expansion(expansions, tmp_path):
    j = isaac.build_job(expansions["bp000"], tmp_path)
    assert j["diffdrive"] == bm.diffdrive_params(expansions["bp000"].text)
    assert j["lift_velocity_limit"] == pytest.approx(0.018)


@pytest.fixture
def kit(expansions, tmp_path):
    job_dir = tmp_path / "sim" / "bp000" / "isaac"
    j = isaac.build_job(expansions["bp000"], job_dir)
    return tmp_path, job_dir, j


def test_collector_without_a_gpu_run_is_blocked(kit):
    root, _, _ = kit
    r = collect_result.collect(root)
    assert r["status"] == "BLOCKED"
    assert {c["check"]: c["status"] for c in r["checks"]} == {"bp000.usd_generated": "BLOCKED", "benchmark.ran": "BLOCKED"}
    assert all(row["verdicts"]["isaac"] == "BLOCKED" for row in r["comparison"])
    assert (root / "isaac_result.json").is_file()


def _fake_gpu_run(root, job_dir, job, roots, dofs, results):
    (job_dir / job["usd"]).write_text("#usda 1.0\n")
    (job_dir / "isaac_import_log.json").write_text(json.dumps({"result": "written", "articulation_roots": roots,
                                                                 "isaac_sim_build": "5.1.0-rc.19"}))
    (job_dir / "check_usd.json").write_text(json.dumps({"summary": {"checks": 120, "failures": 0}, "failures": []}))
    (root / "benchmark_isaac.json").write_text(json.dumps({"schema": bm.SCHEMA, "backend": "isaac", "configuration": "bp000",
                                                           "results": results, "meta": {"dof_count": dofs, "errors": {}}}))


def test_collector_passes_a_complete_run_and_fails_a_split_or_short_one(kit):
    root, job_dir, job = kit
    expected = len(collect_result.moving_joints(job_dir, job))
    assert expected == 21  # bp000 gazebo expansion: 14 arm and 4 finger revolute, 2 continuous wheels, the prismatic lift
    results = {"rest": {"base_height_m": 0.0, "base_pitch_deg": 0.5, "base_roll_deg": 0.0}}
    mujoco = root / "mujoco.json"
    mujoco.write_text(json.dumps({"results": results}))
    _fake_gpu_run(root, job_dir, job, ["/openamrobot/base_footprint"], expected, results)
    r = collect_result.collect(root, mujoco=str(mujoco))
    assert r["status"] == "PASS", r["checks"]
    assert r["configurations"]["bp000"]["usd_sha256"]
    rows = {(x["scenario"], x["metric"]): x for x in r["comparison"]}
    assert rows[("rest", "base_pitch_deg")]["verdicts"]["isaac"] == "PASS"
    _fake_gpu_run(root, job_dir, job, ["/a", "/b"], expected, results)
    assert collect_result.collect(root)["status"] == "FAIL"
    _fake_gpu_run(root, job_dir, job, ["/openamrobot/base_footprint"], expected - 1, results)
    r = collect_result.collect(root)
    assert r["status"] == "FAIL" and any(c["check"] == "bp000.dof_count" and c["status"] == "FAIL" for c in r["checks"])

def test_run_script_installs_xacro_the_same_way_as_ci():
    """When the hash-pinned wheel file exists (upperbody-sw #104), the kit installs xacro from it with --require-hashes,
    exactly as CI does, and does not install the apt build; before that it falls back to the lock's apt build."""
    text = (KIT / "run_on_gpu.sh").read_text()
    assert "--require-hashes --no-deps --ignore-installed -r simulation/requirements-xacro.txt" in text
    assert "if [ -f simulation/requirements-xacro.txt ]" in text
    pip_branch = text.split("if [ -f simulation/requirements-xacro.txt ]; then", 1)[1].split("else", 1)[0]
    assert "ros-jazzy-xacro=" not in pip_branch
