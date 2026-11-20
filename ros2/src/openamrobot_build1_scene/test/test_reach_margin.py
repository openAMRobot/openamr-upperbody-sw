# SPDX-License-Identifier: MIT
"""Reach margin of the scn04 straight reach against the base pose error at the work-shelf station. No ROS, no MoveIt.

Run 44445938978 (control room end to end, station alignment off) stopped the base 44.4 mm farther from the shelf than
stations.work_shelf (6.1 mm right, 0.051 rad), and MoveIt's /compute_cartesian_path covered 0.6508 of the reach
(grasp_reach_cartesian_fraction, allowed 0.999). This test walks the same three waypoints (scenarios.grasp:
pre_grasp, above, hand) in CARTESIAN_STEP_M steps with damped least-squares IK seeded from the previous step, the
URDF joint limits and the orientation the approach ended with, from the approach end point the run logged. It gives
0.6508 = (1 + 20/21) / 3 too: the elbow (openarm_left_joint4) reaches its lower limit 0 rad one step before the end of
the 0.10 m leg along the fingers. Collisions are not modelled, so a collision is not needed to explain the run.

The item's world position comes from the four passing runs (44448676604, 44309791416, 44309804809, 44309817813):
forward kinematics of their logged grasp_reach end points, put back through their logged station errors, agree on
(0.160, -2.835, 0.895) m within 1 mm.

Simulation-derived kinematics only: not a site, robot or safety tolerance.
"""
import math

import numpy as np
import pytest
import xml.etree.ElementTree as ET

from openamrobot_build1_scene import arm_frames as af
from openamrobot_build1_scene.gravity import ARM_JOINTS, GravityModel

JOINTS = ARM_JOINTS["left"]
HAND, TCP = af.HAND_FRAME["left"], af.TCP_FRAME["left"]
STATION = (0.0, -2.085, -math.pi / 2)          # stations.work_shelf, as the mission logs its nav goal
ITEM_WORLD = np.array([0.160, -2.835, 0.895])
LIFT_AT_STOP_0 = 0.0008                         # lift_stop_0_gazebo_position of the runs (L1000)

# (along the heading, + towards the shelf; left; yaw error), as logged: nav_station_error_along_heading,
# nav_station_error_left, -grasp_face_yaw; then the logged grasp_approach and grasp_reach commanded end points.
FAILED = (-0.04444, -0.00614, 0.05118, [-0.74388, -0.22253, 0.35878, 1.35977, -0.42187, -0.37216, 0.27505])
PASSED = {
    44448676604: ((-0.00106, -0.00519, 0.01054), [-0.55795, -0.4321, 0.43849, 1.70055, -0.54828, -0.45963, 0.43052],
                  [-0.98564, -0.26105, 0.52483, 0.91006, -0.48516, -0.17749, 0.17604]),
    44309791416: ((-0.00212, -0.00183, 0.0313), [-0.50365, -0.27125, 0.24444, 1.70191, -0.30162, -0.4797, 0.21755],
                  [-0.95852, -0.11756, 0.32522, 0.84156, -0.24254, -0.09991, 0.09808]),
    44309804809: ((-0.01695, -0.01354, 0.01463), [-0.51721, 0.03563, -0.08599, 1.62622, 0.07888, -0.45022, -0.11146],
                  [-1.01577, 0.00466, -0.08494, 0.67517, 0.06791, 0.00449, -0.0559]),
    44309817813: ((-1e-05, -0.00262, 0.03761), [-0.51733, -0.12211, 0.16777, 1.66437, -0.3183, -0.44149, 0.23201],
                  [-0.93416, -0.04501, 0.20132, 0.83785, -0.28294, -0.05038, 0.09862]),
}


class Arm:
    def __init__(self, urdf):
        self.model = GravityModel(urdf)
        self.limits = {}
        for joint in ET.fromstring(urdf).findall("joint"):
            if joint.get("name") in JOINTS:
                limit = joint.find("limit")
                self.limits[joint.get("name")] = (float(limit.get("lower")), float(limit.get("upper")))
        links, _ = self.frames(np.zeros(7))
        (Rh, ph), (_, pt) = links[HAND], links[TCP]
        self.tcp_offset = tuple(Rh.T @ (pt - ph))

    def frames(self, q):
        positions = dict(zip(JOINTS, q))
        positions["lift_joint"] = LIFT_AT_STOP_0
        return self.model._frames(positions)

    def hand(self, q):
        return self.frames(q)[0][HAND]

    def ik(self, q, point, R_goal, iterations=200):
        q = np.array(q, dtype=float)
        lower = np.array([self.limits[n][0] for n in JOINTS])
        upper = np.array([self.limits[n][1] for n in JOINTS])
        for _ in range(iterations):
            links, joints = self.frames(q)
            R, p = links[HAND]
            error = np.concatenate([point - p, rotation_error(R_goal, R)])
            if np.linalg.norm(error[:3]) < 1e-5 and np.linalg.norm(error[3:]) < 1e-4:
                return q, True
            J = np.array([np.concatenate([np.cross(joints[n][0], p - joints[n][1]), joints[n][0]]) for n in JOINTS]).T
            q = np.clip(q + J.T @ np.linalg.solve(J @ J.T + 1e-3 * np.eye(6), error), lower, upper)
        return q, False


def rotation_error(R_goal, R):
    E = R_goal @ R.T
    angle = math.acos(max(-1.0, min(1.0, (np.trace(E) - 1.0) / 2.0)))
    if angle < 1e-9:
        return np.zeros(3)
    return np.array([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]]) * angle / (2.0 * math.sin(angle))


def quaternion(R):
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    return ((R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w)


def base_pose(along, left, yaw_error):
    x, y, yaw = STATION
    return (x + along * math.cos(yaw) - left * math.sin(yaw), y + along * math.sin(yaw) + left * math.cos(yaw),
            yaw + yaw_error)


def item_in_base(along, left, yaw_error):
    x, y, yaw = base_pose(along, left, yaw_error)
    dx, dy = ITEM_WORLD[0] - x, ITEM_WORLD[1] - y
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c * dx + s * dy, -s * dx + c * dy, ITEM_WORLD[2]])


def reach(arm, q_start, item, held):
    """MoveIt's fraction for scenarios.grasp's reach: (waypoints done + share of the failing one) / waypoints.
    -> (fraction, joint positions where it stopped)."""
    grasp_z = item[2] + af.GRASP_ABOVE_ITEM_CENTRE_M
    tcp = af.tcp_for_item((item[0], item[1], grasp_z), held)
    pre_grasp, above = af.grasp_line((tcp[0], tcp[1], grasp_z + af.REACH_CLEARANCE_M), arm.tcp_offset, held)
    _, hand = af.grasp_line((tcp[0], tcp[1], grasp_z), arm.tcp_offset, held)
    R_goal = np.array(af.quaternion_matrix(held))
    q, previous = np.array(q_start, dtype=float), arm.hand(q_start)[1]
    waypoints = [np.array(pre_grasp), np.array(above), np.array(hand)]
    for i, waypoint in enumerate(waypoints):
        steps = max(1, int(math.ceil(np.linalg.norm(waypoint - previous) / af.CARTESIAN_STEP_M)))
        for k in range(1, steps + 1):
            q, ok = arm.ik(q, previous + (waypoint - previous) * k / steps, R_goal)
            if not ok:
                return (i + k / (steps + 1)) / len(waypoints), q
        previous = waypoint
    return 1.0, q


def approach_start(arm, along, left, yaw_error, seed):
    """IK of scenarios.grasp's approach pose (0.10 m back along the fingers at the grasp height) from `seed`."""
    item, yaw = item_in_base(along, left, yaw_error), -yaw_error
    hand = af.hand_for_tcp_at((item[0], item[1], item[2] + af.GRASP_ABOVE_ITEM_CENTRE_M), arm.tcp_offset, yaw)
    back = af.rotate_z((0.10, 0.0, 0.0), yaw)
    q, ok = arm.ik(seed, np.array([hand[0] - back[0], hand[1] - back[1], hand[2]]),
                   np.array(af.quaternion_matrix(af.side_grasp(yaw))), iterations=500)
    assert ok, "approach pose not reachable"
    return q, item, af.side_grasp(yaw)


@pytest.fixture(scope="module")
def arm(devices_urdf):
    return Arm(devices_urdf)


@pytest.mark.parametrize("run", sorted(PASSED))
def test_the_passing_runs_put_the_item_at_one_world_position(arm, run):
    (along, left, yaw_error), _, q_reach = PASSED[run]
    R, p = arm.hand(q_reach)
    fingers = R @ np.array([0.0, 0.0, -1.0])
    item = p + R @ np.array(arm.tcp_offset) - af.ITEM_DEPTH_BEHIND_TCP_M * fingers
    item[2] -= af.GRASP_ABOVE_ITEM_CENTRE_M
    x, y, yaw = base_pose(along, left, yaw_error)
    world = np.array([x + math.cos(yaw) * item[0] - math.sin(yaw) * item[1],
                      y + math.sin(yaw) * item[0] + math.cos(yaw) * item[1], item[2]])
    assert np.allclose(world, ITEM_WORLD, atol=0.001)


@pytest.mark.parametrize("run", sorted(PASSED))
def test_the_passing_runs_complete_the_reach(arm, run):
    (along, left, yaw_error), q_approach, q_reach = PASSED[run]
    held = quaternion(arm.hand(q_approach)[0])
    fraction, q = reach(arm, q_approach, item_in_base(along, left, yaw_error), held)
    assert fraction == 1.0
    assert np.allclose(q, q_reach, atol=0.03)             # the run's own end point, to the IK solver's redundancy


def test_run_44445938978_stops_where_the_elbow_is_straight(arm):
    along, left, yaw_error, q_approach = FAILED
    held = quaternion(arm.hand(q_approach)[0])
    fraction, q = reach(arm, q_approach, item_in_base(along, left, yaw_error), held)
    assert fraction == pytest.approx(0.6508, abs=1e-3)    # the run: grasp_reach_cartesian_fraction = 0.65079
    assert q[JOINTS.index("openarm_left_joint4")] == pytest.approx(arm.limits["openarm_left_joint4"][0], abs=1e-6)


@pytest.mark.parametrize("left, yaw_error, reachable, unreachable", [
    (0.0, 0.0, -0.047, -0.050),                           # boundary -0.0485 m
    (-0.00614, 0.05118, -0.039, -0.042),                  # the lateral and heading error of run 44445938978: -0.0404 m
    (0.0, 0.08, -0.034, -0.037),                          # the heading budget of station_approach.md: -0.0353 m
])
def test_the_reach_ends_3_5_to_5_cm_behind_the_station(arm, left, yaw_error, reachable, unreachable):
    """The straight reach is the binding move: the budget's 0.04 m along the heading leaves no margin with 0.05 rad and
    is 5 mm short with 0.08 rad. station_approach.md's -0.06 m (pose goals within 0.01 m, TCP on the item centre) does
    not hold for the reach with the item 9 mm into the fingers."""
    seed = PASSED[44309817813][1]
    for along, expected in ((reachable, True), (unreachable, False)):
        q, item, held = approach_start(arm, along, left, yaw_error, seed)
        assert (reach(arm, q, item, held)[0] == 1.0) is expected, along


def test_the_control_room_mission_runs_with_station_alignment():
    """Run 44445938978 ran with BASELINE1_STATION_ALIGNMENT 0: the base stopped where AMCL put it (0.0449 m from the
    station, beyond the reach above). The acceptance runs that passed (44309791416, 44309804809, 44309817813,
    44448676604) ran with the alignment. The control room end to end runs the same mission, so it runs it the same way."""
    import os
    import yaml
    from conftest import REPOSITORY
    with open(os.path.join(REPOSITORY, ".github", "workflows", "control-room-e2e.yml"), encoding="utf-8") as f:
        workflow = yaml.safe_load(f)
    assert str(workflow["env"]["BASELINE1_STATION_ALIGNMENT"]) == "1"
