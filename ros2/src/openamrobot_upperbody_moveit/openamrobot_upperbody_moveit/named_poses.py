"""Named poses read from the SRDF, and the per-hand gripper sign rule (ADR-009 point 5).

The two grippers take opposite-signed GripperCommand positions: left finger_joint1 range [0, 0.7854], right [-0.7854, 0] (upstream's
convention, not ours). A named pose or helper that closes both hands from one number is wrong on one hand WITHOUT failing at the
point it is written, which is the class of defect that reaches a demo. So the closed value of a hand is never a constant here: it is
that hand's own far limit, read from the URDF, and `gripper_sign_problems` is the check that fails the single-number version.
"""
import xml.etree.ElementTree as ET

SIDES = ("left", "right")
GRIPPER_JOINT = {side: f"openarm_{side}_finger_joint1" for side in SIDES}
GRIPPER_GROUP = {side: f"{side}_gripper" for side in SIDES}
ARM_GROUP = {side: f"{side}_arm" for side in SIDES}
ARM_POSES = ("home", "hands_up", "park")       # per arm; `park` is a placeholder copy of `home`
GRIPPER_POSES = ("open", "closed")             # per hand, explicit values


def read_group_states(srdf_text):
    """{(group, state name): {joint: value}} from an SRDF."""
    root = ET.fromstring(srdf_text)
    return {(gs.get("group"), gs.get("name")): {j.get("name"): float(j.get("value")) for j in gs.findall("joint")}
            for gs in root.findall("group_state")}


def urdf_joint_limits(urdf_text):
    """{joint: (lower, upper)} for the revolute joints of a URDF."""
    root = ET.fromstring(urdf_text)
    out = {}
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        if limit is not None and limit.get("lower") is not None:
            out[joint.get("name")] = (float(limit.get("lower")), float(limit.get("upper")))
    return out


def far_limit(side, limits):
    """The end of a hand's own finger range away from zero: where `closed` is. Left +0.7854, right -0.7854, from the URDF."""
    lower, upper = limits[GRIPPER_JOINT[side]]
    return upper if abs(upper) >= abs(lower) else lower


def closed_value(side, limits):
    """The `closed` position for one hand. Per hand and explicit: there is no function of a single number for both."""
    return far_limit(side, limits)


def gripper_sign_problems(states, limits, tolerance=1e-4):
    """Messages for every way the gripper group states break ADR-009 point 5; empty when they are right. States are
    {(group, name): {joint: value}} (read_group_states), limits are {joint: (lower, upper)} (urdf_joint_limits).

    Checks, per hand: `open` and `closed` exist; each value is inside that hand's own URDF range (a left-hand number on the right
    hand is outside it: that is the single-number defect); `closed` is that hand's far limit, not merely something in range; and
    across hands `closed` has opposite signs, because the ranges do."""
    problems = []
    closed = {}
    for side in SIDES:
        joint, group = GRIPPER_JOINT[side], GRIPPER_GROUP[side]
        lower, upper = limits[joint]
        for name in GRIPPER_POSES:
            values = states.get((group, name))
            if values is None:
                problems.append(f"{group}: no group_state `{name}`")
                continue
            if set(values) != {joint}:
                problems.append(f"{group}/{name}: sets {sorted(values)}, expected only {joint}")
                continue
            value = values[joint]
            if not (lower - tolerance <= value <= upper + tolerance):
                problems.append(f"{group}/{name}: {value:+.4f} is outside {side}'s own range [{lower:+.4f}, {upper:+.4f}] of {joint}")
            if name == "closed":
                closed[side] = value
                if abs(value - far_limit(side, limits)) > tolerance:
                    problems.append(f"{group}/closed: {value:+.4f} is not {side}'s far limit {far_limit(side, limits):+.4f}")
    if len(closed) == 2 and closed["left"] * closed["right"] >= 0:
        problems.append(f"closed values {closed['left']:+.4f} (left) and {closed['right']:+.4f} (right) do not have opposite signs: "
                        "one hand's `closed` is another's `open` end")
    return problems
