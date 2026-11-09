# SPDX-License-Identifier: MIT
"""The MoveIt guard's decision, with no ROS: may a plan or an execution proceed on the lift state the host actually has?

MoveIt and robot_state_publisher keep the LAST value of a joint when its publisher stops, so a stopped relay is not enough: something has to
look at the age of the lift_joint sample MoveIt is planning from and refuse when it is missing or old (issue #45, point 3). The guard watches
/joint_states itself (the topic move_group reads) rather than trusting the relay's word, and adds the relay's own diagnostic as the reason.
"""
from dataclasses import dataclass

LIFT_JOINT = "lift_joint"
MAX_SAMPLE_AGE_S = 0.6          # the /base/aux_status stale time: the relay publishes on each fresh message (5 Hz), so a sample older than this is stale
REASON_SOURCE = "lift_guard"


@dataclass(frozen=True)
class GuardDecision:
    allowed: bool
    reason: str

    def __bool__(self):
        return self.allowed


class LiftSampleGuard:
    """Tracks when a lift_joint sample last arrived on /joint_states (LOCAL receipt time) and what the relay says about the lift."""

    def __init__(self, max_sample_age_s=MAX_SAMPLE_AGE_S):
        self.max_sample_age_s = max_sample_age_s
        self.last_sample = None       # local time of the last /joint_states message that carried a finite lift_joint
        self.relay_text = ""          # the relay's latest diagnostic message, for the reason only

    def on_joint_state(self, names, positions, now):
        """Record a /joint_states message. A message without lift_joint, or with a non-finite value, does not refresh the lift."""
        import math
        for name, position in zip(names, positions):
            if name == LIFT_JOINT and math.isfinite(position):
                self.last_sample = now
                return

    def on_relay_status(self, text):
        self.relay_text = text

    def check(self, now):
        if self.last_sample is None:
            return GuardDecision(False, self._reason("no lift_joint state has been received on /joint_states"))
        age = now - self.last_sample
        if age > self.max_sample_age_s:
            return GuardDecision(False, self._reason(f"lift_joint state is stale ({age:.2f} s old, limit {self.max_sample_age_s:g} s)"))
        return GuardDecision(True, "lift state fresh")

    def _reason(self, base):
        return f"{base}; relay: {self.relay_text}" if self.relay_text else base


# For the launch that starts move_group behind the guard: the three names the guard serves, moved aside on move_group's side.
GUARD_REMAPPINGS = [("move_action", "move_action_unguarded"), ("execute_trajectory", "execute_trajectory_unguarded"),
                    ("plan_kinematic_path", "plan_kinematic_path_unguarded")]
