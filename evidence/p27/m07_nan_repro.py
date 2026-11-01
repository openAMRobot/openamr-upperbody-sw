# SPDX-License-Identifier: MIT
"""Mohammad's two M07 reproductions, run through the real helpers the way the launch test's teardown combines them.

    python3 -B evidence/p27/m07_nan_repro.py <path to ros2/src/openamrobot_upperbody_bringup/test>

BEFORE (ba5b4ea): both print PASS. AFTER the P27 change this script prints the evaluate_m07 verdict, which is FAIL for both.
No simulator: readings are synthetic, mid-range for every expected joint, so the only defect is the NaN.
"""
import os
import sys

sys.path.insert(0, sys.argv[1])
os.environ.setdefault("OPENAMR_A7", "1")
import a7_support  # noqa: E402
import test_m07_rest_position_margin as m  # noqa: E402

xml = m.model_xml()
limits = m.joint_limits_with_names(xml)
expected, _ = a7_support.expected_rest_joints(xml, m.NOT_MEASURED)
mid = {j: (limits[j][0] + limits[j][1]) / 2 for j in expected}
nan = float("nan")


def verdict(readings):
    if hasattr(a7_support, "evaluate_m07"):
        return a7_support.evaluate_m07(readings, expected, limits, 1e-3, 50)["result"]
    rows = m.rest_rows(readings, limits, 1e-3)
    cov = a7_support.coverage_problems(readings, expected, 50)
    failing, warned = m.classify(rows, 1e-3)
    return "FAIL" if (not rows or cov["missing"] or cov["too_few_samples"] or failing) else ("WARN" if warned else "PASS")


print("code:", "evaluate_m07" if hasattr(a7_support, "evaluate_m07") else "ba5b4ea logic (rest_rows + coverage_problems + classify)")
print("a) 50 NaN per expected joint                     ->", verdict({j: [nan] * 50 for j in expected}))
print("b) 49 finite (mid-range) + 1 NaN last, per joint ->", verdict({j: [mid[j]] * 49 + [nan] for j in expected}))
print("b') 1 NaN first + 49 finite, per joint            ->", verdict({j: [nan] + [mid[j]] * 49 for j in expected}))
