# SPDX-License-Identifier: MIT
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


# --- named poses that coincide with a joint limit (P18's W03; evidence/w03/, not a registered check since P19) -------------------------
# Information only, gating nothing. A joint commanded to a value that IS one of its limits is a fact about the pose definitions (upstream's `home` is
# joint 4 at its lower limit 0, a gripper's `open` is the end of its range), not a defect of any body: it is true of the old mast body and of
# the lift body alike, and the report is for whichever SRDF and URDF it is given. It is not M07, which measures a simulated rest position
# against a margin datum; it registers nothing while M07 exists.
ON_LIMIT_TOLERANCE = 1e-6      # rad. NOT a margin: the numeric slack within which two numbers written in files are the same number


def poses_on_limits(states, limits, tolerance=ON_LIMIT_TOLERANCE):
    """Every (group, named pose, joint) whose value equals one of the joint's URDF limits to within `tolerance`, with the distance.
    `states` is read_group_states(srdf), `limits` is urdf_joint_limits(urdf). A value past a limit has a negative distance and is listed
    too. Joints with no URDF limit are not in `limits` and are listed under the second return value, never silently skipped."""
    rows, unlimited = [], []
    for (group, pose), joints in sorted(states.items()):
        for joint, value in sorted(joints.items()):
            if joint not in limits:
                unlimited.append({"group": group, "pose": pose, "joint": joint})
                continue
            lower, upper = limits[joint]
            to_lower, to_upper = value - lower, upper - value
            distance, which = (to_lower, "lower") if to_lower <= to_upper else (to_upper, "upper")
            if abs(distance) <= tolerance:
                rows.append({"group": group, "pose": pose, "joint": joint, "value_rad": value, "limit": which,
                             "limit_rad": lower if which == "lower" else upper, "distance_rad": distance})
    return rows, unlimited


def on_limit_report(srdf_text, urdf_text, tolerance=ON_LIMIT_TOLERANCE):
    rows, unlimited = poses_on_limits(read_group_states(srdf_text), urdf_joint_limits(urdf_text), tolerance)
    return {"tolerance_rad": tolerance, "poses_on_a_limit": rows, "joints_on_a_limit": sorted({r["joint"] for r in rows}),
            "named_pose_joints_without_a_urdf_limit": unlimited}


if __name__ == "__main__":
    import argparse
    import json
    parser = argparse.ArgumentParser(description="named poses of an SRDF that coincide with a joint limit of a URDF")
    parser.add_argument("srdf")
    parser.add_argument("urdf")
    args = parser.parse_args()
    with open(args.srdf) as f_srdf, open(args.urdf) as f_urdf:
        print(json.dumps(on_limit_report(f_srdf.read(), f_urdf.read()), indent=1))
