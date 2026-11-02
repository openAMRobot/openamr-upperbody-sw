# SPDX-License-Identifier: MIT
"""G01 test_g01_6's rule for who reports lift_joint (issue #17, owner decision 1 Nov 2026), tested both ways without Gazebo.

mock / gazebo: joint_state_broadcaster (/arm/joint_states) reports lift_joint exactly once in every message, the base source never.
real: no interface, so neither source reports it and only the merger's default does (the expectation before #17).
The rule must fail when lift_joint is reported twice, or not at all."""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_test_support import lift_source_problems  # noqa: E402

WHEELS = ["left_wheel_joint", "right_wheel_joint"]
ARMS = ["openarm_left_joint1", "openarm_right_joint1"]
MERGED = WHEELS + ARMS + ["lift_joint"]


@pytest.mark.parametrize("profile", ["mock", "gazebo"])
def test_simulated_lift_reported_once_by_the_broadcaster_passes(profile):
    assert lift_source_problems(profile, [WHEELS] * 3, [ARMS + ["lift_joint"]] * 3, MERGED) == []


@pytest.mark.parametrize("profile", ["mock", "gazebo"])
def test_simulated_lift_reported_twice_fails(profile):
    twice_in_one = lift_source_problems(profile, [WHEELS], [ARMS + ["lift_joint", "lift_joint"]], MERGED)
    assert any("more than once" in p for p in twice_in_one), twice_in_one
    by_both_sources = lift_source_problems(profile, [WHEELS + ["lift_joint"]], [ARMS + ["lift_joint"]], MERGED)
    assert any("base source" in p for p in by_both_sources), by_both_sources


@pytest.mark.parametrize("profile", ["mock", "gazebo"])
def test_simulated_lift_not_reported_fails(profile):
    by_nobody = lift_source_problems(profile, [WHEELS], [ARMS], MERGED)
    assert any("does not report lift_joint" in p for p in by_nobody), by_nobody
    missing_in_one = lift_source_problems(profile, [WHEELS], [ARMS + ["lift_joint"], ARMS], MERGED)
    assert any("does not report lift_joint" in p for p in missing_in_one), missing_in_one
    not_merged = lift_source_problems(profile, [WHEELS], [ARMS + ["lift_joint"]], WHEELS + ARMS)
    assert any("not on /joint_states" in p for p in not_merged), not_merged


def test_real_keeps_the_old_expectation():
    assert lift_source_problems("real", [WHEELS], [ARMS], MERGED) == []
    reported = lift_source_problems("real", [WHEELS], [ARMS + ["lift_joint"]], MERGED)
    assert any("other than the merger's default" in p for p in reported), reported
    assert lift_source_problems("real", [WHEELS], [ARMS], WHEELS + ARMS)
