# SPDX-License-Identifier: MIT
"""M06 (adopted 7 Oct, VERIFICATION.md): the named-pose plans NEED some acceleration limit and do not DEPEND on the values we invented (ADR-009).

Runs scripts/limits_sensitivity.py: all twelve named-pose plans (both arms, both grippers) under the configured limits scaled by 1, 0.1, 0.01
and 0.001, and under no acceleration limits at all (upstream's own state, manipulation#10). The A7 stop condition is "planning succeeds only
because the acceleration limits we invented are generous": it would show here as a plan that fails under a tight scaling."""
import json
import os
import subprocess
import sys
import tempfile

from ament_index_python.packages import get_package_prefix

from conftest import write_sidecar

SCRIPT = os.path.join(get_package_prefix("openamrobot_upperbody_moveit"), "lib", "openamrobot_upperbody_moveit", "limits_sensitivity.py")


def test_m06_plans_succeed_at_every_scaling_of_the_configured_limits_and_fail_with_none():
    env = {**os.environ, "ROS_DOMAIN_ID": str(__import__("random").randint(201, 232)), "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
           "PYTHONDONTWRITEBYTECODE": "1"}
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, "sensitivity.json")
        done = subprocess.run([sys.executable, SCRIPT, "--json", out], capture_output=True, text=True, timeout=300, env=env)
        assert os.path.isfile(out), f"limits_sensitivity.py produced no result (exit {done.returncode}):\n{done.stdout[-1500:]}\n{done.stderr[-800:]}"
        with open(out) as f:
            result = json.load(f)
    load_bearing = [v["variant"] for v in result["variants"] if v["plans_ok"] != v["plans"]]
    needs_some = result["without_limits"]["plans_ok"] < result["without_limits"]["plans"]
    write_sidecar("M06", "limits_sensitivity", "named-pose plans need some acceleration limit and do not depend on its value",
                  "FAIL" if load_bearing else "PASS",
                  {"verdict": result["verdict"], "conclusion": result["conclusion"], "base_plate": result["base_plate"], "velocity_scaling": result["velocity_scaling"],
                   "variants": [{k: v[k] for k in ("variant", "acceleration_scaling", "plans", "plans_ok", "total_duration_s")}
                                for v in result["variants"]],
                   "without_limits": {k: result["without_limits"][k] for k in ("variant", "plans", "plans_ok")},
                   "plans_that_depend_on_the_values": load_bearing, "plans_need_some_limit_to_exist": needs_some,
                   "meaning": "the acceleration limits are engineering defaults tagged placeholder (ADR-009); the plans succeed with them scaled "
                              "down a thousandfold, so they are not what makes planning succeed, they are only what lets MoveIt's time "
                              "parameterisation run at all"})
    # The conclusion has two halves or it is an overclaim: what the sensitivity retires, and that the limits stay placeholders.
    assert "retires the A7 stop condition" in result["conclusion"] and "NOT make the limits any less of a placeholder" in result["conclusion"]
    assert not load_bearing, f"the plans depend on the invented values: fail under {load_bearing}"
    assert needs_some, "with no acceleration limits the plans still succeed: the premise of manipulation#10 no longer holds; re-read ADR-009"
