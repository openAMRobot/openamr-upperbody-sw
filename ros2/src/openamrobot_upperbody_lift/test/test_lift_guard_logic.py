# SPDX-License-Identifier: MIT
"""The MoveIt guard's decision, no ROS: refuse on a missing or stale lift_joint sample, say why."""
import math

from conftest import FakeClock

from openamrobot_upperbody_lift.guard_logic import LiftSampleGuard


def test_refuses_when_no_lift_state_was_ever_received():
    guard = LiftSampleGuard()
    decision = guard.check(1000.0)
    assert not decision and "no lift_joint state" in decision.reason


def test_allows_on_a_fresh_sample():
    guard = LiftSampleGuard()
    guard.on_joint_state(["lift_joint"], [0.1], 1000.0)
    assert guard.check(1000.3)


def test_refuses_on_a_stale_sample_and_reports_the_age():
    guard = LiftSampleGuard()
    guard.on_joint_state(["lift_joint"], [0.1], 1000.0)
    decision = guard.check(1000.7)
    assert not decision and "stale" in decision.reason and "0.70" in decision.reason


def test_other_joints_do_not_refresh_the_lift():
    guard = LiftSampleGuard()
    guard.on_joint_state(["lift_joint"], [0.1], 1000.0)
    guard.on_joint_state(["openarm_left_joint1"], [0.3], 1000.6)         # arms keep publishing: the lift is still old
    assert not guard.check(1000.7)


def test_a_non_finite_lift_value_does_not_refresh_the_lift():
    guard = LiftSampleGuard()
    guard.on_joint_state(["lift_joint"], [0.1], 1000.0)
    guard.on_joint_state(["lift_joint"], [math.nan], 1000.5)
    assert not guard.check(1000.7)


def test_the_relay_reason_is_appended():
    guard = LiftSampleGuard()
    guard.on_relay_status("lift_position_valid is false")
    decision = guard.check(1000.0)
    assert not decision and decision.reason.endswith("relay: lift_position_valid is false")


def test_recovers_when_fresh_samples_return():
    clock = FakeClock()
    guard = LiftSampleGuard()
    guard.on_joint_state(["lift_joint"], [0.1], clock())
    clock.advance(1.0)
    assert not guard.check(clock())
    guard.on_joint_state(["lift_joint"], [0.1], clock())
    assert guard.check(clock())


def test_the_remappings_cover_every_guarded_name():
    from openamrobot_upperbody_lift.guard_logic import GUARD_REMAPPINGS
    assert dict(GUARD_REMAPPINGS) == {"move_action": "move_action_unguarded", "execute_trajectory": "execute_trajectory_unguarded",
                                      "plan_kinematic_path": "plan_kinematic_path_unguarded"}
