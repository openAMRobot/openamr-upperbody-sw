# SPDX-License-Identifier: MIT
"""The Simulators workflow: without a GPU runner the Isaac GPU job is skipped, not red, and BLOCKED is reported."""
import pathlib

import yaml

WORKFLOW = pathlib.Path(__file__).resolve().parents[2] / ".github/workflows/simulators.yml"


def test_ci_skips_the_gpu_job_without_a_gpu_runner_and_reports_blocked():
    """Without a GPU runner the `isaac` job is skipped (not red on every pull request), the `mujoco` job, which runs the CPU Isaac
    checks, reports the USD import as BLOCKED, and the GPU job never falls back to a hosted runner."""
    wf = yaml.safe_load(WORKFLOW.read_text())
    isaac = wf["jobs"]["isaac"]
    assert isaac.get("if") == "vars.ISAAC_GPU_RUNNER != ''"
    assert isaac["runs-on"] == "${{ vars.ISAAC_GPU_RUNNER }}"
    report = [s for s in wf["jobs"]["mujoco"]["steps"] if "BLOCKED" in (s.get("name") or "")]
    assert len(report) == 1 and "vars.ISAAC_GPU_RUNNER == ''" in report[0]["if"]
    assert "::warning" in report[0]["run"] and "GITHUB_STEP_SUMMARY" in report[0]["run"]
