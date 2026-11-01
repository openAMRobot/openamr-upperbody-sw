# SPDX-License-Identifier: MIT
"""M07 rejects non-finite joint positions and cannot pass on them (PR #14 review, Mohammad, twice; P27). No simulator, no ROS graph.

ALEX'S RULE, quoted from the comment posted on PR #14 by BotshareAI on 20 Oct 2026 ("Proposed M07 rest-state rule (Alex decision, 16 Oct 2026). Om, please
confirm or comment."; once Om confirms it goes into the decisions register as M07-REST-STATE, and it is a proposal until then):

  1. Scope: derive the expected set from the combined model: all 16 limited arm and gripper joints under ros2_control. lift_joint and the mimic
     finger_joint2 joints are recorded as not independently measured and are excluded from the measured set.
  2. Evidence: use raw /arm/joint_states samples during the six-second simulated-time measurement window, after the six-second settle period. Each
     expected joint must have at least 50 samples. A missing joint, fewer than 50 samples, or any non-finite position value (NaN or infinity) is FAIL;
     invalid values must not count as valid samples.
  3. Margin: each measured joint must remain at least 0.001 rad inside both URDF limits throughout the window. The margin and clamp epsilon are unchanged.
  4. Exception: only openarm_{left,right}_finger_joint1 and openarm_{left,right}_joint4 may receive WARN when their measured positions violate the 0.001
     rad margin at their inherited upstream limit. A reading more than 0.001 rad beyond either physical limit is FAIL, including for these four joints.
     All other joints that violate the margin are FAIL.
  5. WARN is a recorded limitation, not acceptance. It remains until the upstream limits or rest pose are corrected and the real arms' rest state is
     checked on hardware during integration.
  6. Evidence handling: retain invalid and incomplete measurements in the sidecar and report them as FAIL; they must never be converted into PASS or WARN.
  ("Tushar, point 2 covers Mohamed's open finding on non-finite values.")

Before (ba5b4ea): a NaN compares false with everything, so 50 NaNs per expected joint gave a coverage of 50, `min`/`max` of NaN, a distance of NaN,
`within_margin` False and a PASS (reproduction a); 49 finite readings and one NaN counted 50 and passed too (reproduction b).
The helper tests need only the installed packages; the model tests expand the real URDF.
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import a7_support as a7  # noqa: E402

MIN = 50
MARGIN = 1e-3
NAN, INF, NINF = float("nan"), float("inf"), float("-inf")
LIFT = {"lift_joint": "not under ros2_control"}
UPSTREAM_LIMIT_JOINTS = ["openarm_left_finger_joint1", "openarm_right_finger_joint1", "openarm_left_joint4", "openarm_right_joint4"]


@pytest.fixture(scope="module")
def model():
    import xacro
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory("openamrobot_upperbody_description"), "urdf", "combined_robot.xacro")
    return xacro.process_file(path, mappings={"hardware_profile": "gazebo"}).toxml()


@pytest.fixture(scope="module")
def expected(model):
    return a7.expected_rest_joints(model, LIFT)[0]


@pytest.fixture(scope="module")
def limits(model):
    from openamrobot_upperbody_moveit.named_poses import urdf_joint_limits
    return urdf_joint_limits(model)


@pytest.fixture(scope="module")
def mid(expected, limits):
    """A safe rest value for every expected joint: the middle of its range, far from both limits."""
    return {j: (limits[j][0] + limits[j][1]) / 2 for j in expected}


def evaluate(readings, expected, limits):
    return a7.evaluate_m07(readings, expected, limits, MARGIN, MIN)


def good(expected, mid, n=MIN):
    return {j: [mid[j]] * n for j in expected}


def test_the_baseline_is_a_pass_with_nothing_at_a_limit(expected, limits, mid):
    """The control for every case below: finite, covered, away from every limit. Without it a FAIL below could be the fixture's doing."""
    result = evaluate(good(expected, mid), expected, limits)
    assert result["result"] == "PASS" and result["problems"] == [] and result["invalid_readings"] == {}


# --- Mohammad's two reproductions -------------------------------------------------------------------------------------------------------------

def test_reproduction_a_fifty_nans_per_expected_joint_is_a_fail(expected, limits):
    result = evaluate({j: [NAN] * MIN for j in expected}, expected, limits)
    assert result["result"] == "FAIL"
    assert sorted(result["coverage"]["missing"]) == sorted(expected)               # no joint has one valid reading
    assert result["rows"] == []                                                    # no extremum was taken from a NaN
    assert set(result["invalid_readings"]) == set(expected)
    assert all(e["count"] == MIN and e["by_kind"] == {"nan": MIN} for e in result["invalid_readings"].values())
    assert all(n == 0 for n in result["valid_samples_per_joint"].values()) and all(n == MIN for n in result["raw_samples_per_joint"].values())


@pytest.mark.parametrize("position", ["last", "first", "middle"])
def test_reproduction_b_forty_nine_finite_and_one_nan_per_joint_is_a_fail(expected, limits, mid, position):
    """49 finite + 1 NaN is 50 readings and 49 valid ones: fewer than the 50 the check needs."""
    def series(j):
        values = [mid[j]] * 49
        values.insert({"last": 49, "first": 0, "middle": 24}[position], NAN)
        return values
    result = evaluate({j: series(j) for j in expected}, expected, limits)
    assert result["result"] == "FAIL"
    assert result["coverage"]["too_few_samples"] == {j: 49 for j in expected} and result["coverage"]["missing"] == []
    assert all(e["count"] == 1 for e in result["invalid_readings"].values())
    assert all(r["readings"] == 49 for r in result["rows"])                       # the extrema came from the 49 finite readings only


# --- the rest of the contract -----------------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("bad,kind", [(NAN, "nan"), (INF, "+inf"), (NINF, "-inf")])
def test_one_non_finite_reading_among_plenty_of_finite_ones_is_still_a_fail(expected, limits, mid, bad, kind):
    """Mixed: 100 finite readings per joint (twice the minimum) and one non-finite on one joint. Coverage is complete, and it is still a FAIL:
    the invalid reading is kept in the result, not averaged away."""
    readings = good(expected, mid, 100)
    joint = "openarm_left_joint2"
    readings[joint][37] = bad
    result = evaluate(readings, expected, limits)
    assert result["result"] == "FAIL"
    assert result["coverage"] == {"missing": [], "too_few_samples": {}}
    assert result["invalid_readings"] == {joint: {"count": 1, "by_kind": {kind: 1}, "indices": [37]}}
    assert result["valid_samples_per_joint"][joint] == 99 and result["valid_samples_per_joint"]["openarm_left_joint3"] == 100
    row = next(r for r in result["rows"] if r["joint"] == joint)
    assert row["min_rad"] == row["max_rad"] == mid[joint] and math.isfinite(row["distance_to_nearest_limit_rad"])


@pytest.mark.parametrize("bad", [INF, NINF])
def test_an_infinity_is_rejected_like_a_nan_and_never_taken_as_a_distance(expected, limits, mid, bad):
    result = evaluate({j: [bad] * MIN for j in expected}, expected, limits)
    assert result["result"] == "FAIL" and result["rows"] == []
    assert all(e["by_kind"] == {("+inf" if bad > 0 else "-inf"): MIN} for e in result["invalid_readings"].values())


def test_a_non_finite_value_never_counts_toward_the_minimum(expected):
    """The count is of finite readings, whether or not the caller split them first."""
    readings = {j: [0.1] * 49 + [NAN, INF, NINF] for j in expected}
    assert a7.coverage_problems(readings, expected, MIN)["too_few_samples"] == {j: 49 for j in expected}
    assert a7.coverage_problems({"a": [NAN] * 100}, ["a"], MIN) == {"missing": ["a"], "too_few_samples": {}}


def test_mixed_finite_and_non_finite_keeps_every_rejected_reading_with_its_position(expected, limits, mid):
    readings = good(expected, mid, 60)
    readings["openarm_right_joint7"][3] = NAN
    readings["openarm_right_joint7"][4] = INF
    readings["openarm_right_joint7"][59] = NINF
    result = evaluate(readings, expected, limits)
    assert result["result"] == "FAIL"
    assert result["invalid_readings"]["openarm_right_joint7"] == {"count": 3, "by_kind": {"nan": 1, "+inf": 1, "-inf": 1}, "indices": [3, 4, 59]}
    assert result["raw_samples_per_joint"]["openarm_right_joint7"] == 60 and result["valid_samples_per_joint"]["openarm_right_joint7"] == 57
    assert any("non-finite" in p for p in result["problems"])


def test_a_missing_expected_joint_is_a_fail(expected, limits, mid):
    readings = good(expected, mid)
    del readings["openarm_right_finger_joint1"]
    result = evaluate(readings, expected, limits)
    assert result["result"] == "FAIL" and result["coverage"]["missing"] == ["openarm_right_finger_joint1"] and result["invalid_readings"] == {}


def test_a_joint_with_too_few_finite_readings_is_a_fail(expected, limits, mid):
    readings = good(expected, mid)
    readings["openarm_left_joint1"] = readings["openarm_left_joint1"][:49]
    result = evaluate(readings, expected, limits)
    assert result["result"] == "FAIL" and result["coverage"]["too_few_samples"] == {"openarm_left_joint1": 49}


def test_a_non_finite_reading_on_a_joint_that_is_not_expected_is_still_a_fail(expected, limits, mid):
    readings = good(expected, mid)
    readings["lift_joint"] = [NAN] * MIN
    assert evaluate(readings, expected, limits)["result"] == "FAIL"


def test_the_sidecar_text_names_the_non_finite_joints(expected, limits):
    result = evaluate({j: [NAN] * MIN for j in expected}, expected, limits)
    assert result["problems"] and result["problems"][0].startswith("non-finite readings (FAIL): ")


# --- the rest of Alex's rule, so the whole check is pinned and not only the new part ------------------------------------------------------------

def test_the_expected_set_is_the_sixteen_limited_joints_and_the_exclusions_are_recorded(model):
    expected, also = a7.expected_rest_joints(model, LIFT)
    assert len(expected) == 16 and "lift_joint" not in expected
    assert sorted(also) == ["openarm_left_finger_joint2", "openarm_right_finger_joint2"]
    assert set(UPSTREAM_LIMIT_JOINTS) <= set(expected) and set(a7.AT_UPSTREAM_LIMIT) == set(UPSTREAM_LIMIT_JOINTS)


@pytest.mark.parametrize("joint", UPSTREAM_LIMIT_JOINTS)
@pytest.mark.parametrize("offset", [0.0, 2e-4, -2e-4, -MARGIN])
def test_the_four_upstream_limit_joints_at_their_limit_are_a_warn_never_a_pass(expected, limits, mid, joint, offset):
    """At the limit, inside it by less than the margin, or past it by no more than the margin: WARN. (-MARGIN is 'not MORE than' the margin.)"""
    limit, inside = 0.0, (1 if limits[joint][0] == 0.0 else -1)                       # all four have a limit at 0; `inside` points into the range
    readings = good(expected, mid)
    readings[joint] = [limit + inside * offset] * MIN
    result = evaluate(readings, expected, limits)
    assert result["result"] == "WARN" and result["failing_joints"] == [] and set(result["warned_joints"]) == {joint}


@pytest.mark.parametrize("joint", UPSTREAM_LIMIT_JOINTS)
def test_more_than_the_margin_beyond_a_limit_is_a_fail_for_the_four_as_for_any_joint(expected, limits, mid, joint):
    limit, inside = 0.0, (1 if limits[joint][0] == 0.0 else -1)
    readings = good(expected, mid)
    readings[joint] = [limit - inside * (MARGIN + 1e-6)] * MIN
    result = evaluate(readings, expected, limits)
    assert result["result"] == "FAIL" and result["failing_joints"] == [joint]


def test_any_other_joint_within_the_margin_of_a_limit_is_a_fail_and_a_far_one_is_not(expected, limits, mid):
    joint = "openarm_left_joint2"
    readings = good(expected, mid)
    readings[joint] = [limits[joint][0] + 5e-4] * MIN
    assert evaluate(readings, expected, limits)["result"] == "FAIL"
    readings[joint] = [limits[joint][0] - 5e-4] * MIN                               # past a limit, not one of the four
    assert evaluate(readings, expected, limits)["result"] == "FAIL"
    readings[joint] = [limits[joint][0] + 2 * MARGIN] * MIN                         # outside the margin
    assert evaluate(readings, expected, limits)["result"] == "PASS"


@pytest.mark.parametrize("joint", UPSTREAM_LIMIT_JOINTS)
def test_the_far_end_of_the_range_is_not_an_inherited_limit_so_it_is_a_fail_even_for_the_four(expected, limits, mid, joint):
    """Point 4 says WARN `at their inherited upstream limit`: the zero end. A joint4 resting at 2.4435 or a finger at +-0.7854 is within the margin of a
    limit it did not inherit as a rest pose, and is FAIL like any other joint."""
    lower, upper = limits[joint]
    far = upper if lower == 0.0 else lower
    inside = -1 if far == upper else 1
    for offset in (0.0, 2e-4):
        readings = good(expected, mid)
        readings[joint] = [far + inside * offset] * MIN
        result = evaluate(readings, expected, limits)
        assert result["result"] == "FAIL" and result["failing_joints"] == [joint] and result["warned_joints"] == {}
