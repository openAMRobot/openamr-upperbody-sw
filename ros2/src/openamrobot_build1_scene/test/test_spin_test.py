# SPDX-License-Identifier: MIT
"""EXPERIMENT (diag/spin-test scratch branch): turns in place and straight drives in firmware_sil, one model variant per
run (BUILD1_SPIN_VARIANT, spin_variants.py). Headless, no Nav2, no MoveIt, no rendered sensor, its own ROS domain.
The numbers go to BUILD1_LOG_DIR/spin_test_<variant>.json. Simulation evidence only. BLOCKED is not a pass.
"""
import os
import sys
import unittest

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import build1_harness  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    return build1_harness.description(nav2="false", moveit="false", sensors="false", extra_sensors="false")


class TestSpinTest(unittest.TestCase):
    def test_spin_test(self):
        if build1_harness.STATE["blocked"]:
            self.skipTest(build1_harness.STATE["blocked"])
        from openamrobot_build1_scene import rig, spin_test
        variant = os.environ.get("BUILD1_SPIN_VARIANT", "")
        self.assertTrue(variant, "BUILD1_SPIN_VARIANT is not set")
        os.environ.setdefault("BUILD1_LOG_DIR", os.path.join(os.getcwd(), "build1_logs"))
        result, reason, path = rig.run(f"spin_test_{variant}", spin_test.run, needs_devices=True, uses_base=True, arms=True)
        print(f"BUILD1 spin_test_{variant}: {result}. Log: {path}")
        if result == "BLOCKED":
            self.skipTest(f"BLOCKED: {reason}")
        self.assertEqual(result, "PASS", reason)
