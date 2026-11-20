#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Run the backend benchmark scenarios (scenarios.yaml) in Gazebo Harmonic and write the result JSON.

Starts `sim.launch.py headless:=true` (the combined robot, bp000, the launch's default engine), waits for its controllers, then runs
every scenario in order in that one simulation. Commands go through the interfaces the twin uses: FollowJointTrajectory on
left_arm_controller and lift_controller, Twist on /cmd_vel to the DiffDrive plugin. Ground truth is Gazebo's own pose topic
(/world/empty/dynamic_pose/info, read with `gz topic`), never /odom or /joint_states, except joint positions, which come from
/joint_states (joint_state_broadcaster, the controllers' own state).

A controller's action result is recorded and never taken as motion. Each commanded joint move is judged on measured travel
(openamr_sim/benchmark.py execution): /joint_states, and for the lift also the carriage link pose from Gazebo. In the bare
simulation (sim.launch.py) lift_controller commands the position interface of gz_ros2_control; on DART the carriage resting on
its lower limit (L1000) does not leave it and the controller, which has no goal tolerance, still reports SUCCESSFUL
(docs/build1_lift_and_base_diagnosis.md on integration/baseline1). That result is now a FAIL of the Gazebo lift_move scenario.

Direct /cmd_vel: the base scenarios publish to the DiffDrive plugin's input with no command chain in the loop. This is an isolated
physics benchmark only, labelled so in every result document, and the runner refuses to start when the graph runs any node of the
approved command chain. Whole-robot acceptance commands the base through that chain, never through this runner.

  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  xvfb-run -a python3 simulation/benchmark/gazebo_benchmark.py --out build/bench/benchmark_gazebo.json

Simulation evidence only: not hardware, physical acceptance or safety evidence.
"""
import argparse
import json
import math
import os
import pathlib
import random
import signal
import subprocess
import sys
import threading
import time

os.environ.setdefault("ROS_DOMAIN_ID", str(random.randint(101, 200)))
os.environ.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
os.environ.setdefault("GZ_PARTITION", f"bench_{os.getpid()}")

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from openamr_sim import benchmark as bm  # noqa: E402

WORLD = "empty"
MODEL = "openamrobot"
BOX = "bench_box"
CARRIAGE = "lift_carriage_link"       # child of lift_joint: its pose in base_footprint is the simulator's own lift height
SUCCESSFUL = 0                        # control_msgs/action/FollowJointTrajectory result error_code SUCCESSFUL
LEFT = [f"openarm_left_joint{k}" for k in range(1, 8)]
CONTROLLERS = ("joint_state_broadcaster", "left_arm_controller", "lift_controller")


# ------------------------------------------------------------------------------------------------------------- poses
def first_json(text):
    stripped = text.lstrip()
    if not stripped:
        raise ValueError("gz topic printed no message")
    return json.JSONDecoder().raw_decode(stripped)[0]


def pose_of(p):
    pos = p.get("position") or {}
    o = p.get("orientation")
    q = (1.0, 0.0, 0.0, 0.0) if not o else (o.get("w", 0.0), o.get("x", 0.0), o.get("y", 0.0), o.get("z", 0.0))
    return [pos.get("x", 0.0), pos.get("y", 0.0), pos.get("z", 0.0)], q


def rot(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def compose(Ra, pa, Rb, pb):
    R = [[sum(Ra[i][k] * Rb[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    p = [pa[i] + sum(Ra[i][k] * pb[k] for k in range(3)) for i in range(3)]
    return R, p


def read_poses(timeout=30):
    out = subprocess.run(["gz", "topic", "-e", "-t", f"/world/{WORLD}/dynamic_pose/info", "-n", "1", "--json-output"],
                         capture_output=True, text=True, timeout=timeout).stdout
    msg = first_json(out)
    stamp = (msg.get("header") or {}).get("stamp") or {}
    t = float(stamp.get("sec", 0)) + 1e-9 * float(stamp.get("nsec", 0))
    return t, {p.get("name"): pose_of(p) for p in msg.get("pose", [])}


def base_pose(poses):
    """World pose of base_footprint: the model pose, composed with base_footprint's pose in the model when the topic lists it."""
    p, q = poses[MODEL]
    R = rot(q)
    if "base_footprint" in poses:
        pb, qb = poses["base_footprint"]
        R, p = compose(R, p, rot(qb), pb)
    return R, p


def link_in_base(poses, link):
    """A link's position in base_footprint. Link poses on the topic are relative to the model."""
    pl, ql = poses[link]
    if "base_footprint" in poses:
        pb, qb = poses["base_footprint"]
        Rb = rot(qb)
        return bm.in_base(Rb, pb, pl)
    return list(pl)


class PoseSampler(threading.Thread):
    """Reads the pose topic again and again in the background, so a turn can be unwrapped from samples less than pi apart."""

    def __init__(self):
        super().__init__(daemon=True)
        self.samples, self.stop, self.errors = [], False, 0

    def run(self):
        while not self.stop:
            try:
                self.samples.append(read_poses(timeout=10))
            except Exception:  # noqa: BLE001  (a missed sample is counted, not fatal)
                self.errors += 1
                time.sleep(0.2)

    def between(self, t0, t1):
        return [(t, p) for t, p in list(self.samples) if t0 <= t <= t1 and MODEL in p]


# ------------------------------------------------------------------------------------------------------------- runner
class Bench:
    def __init__(self, spec, urdf_text):
        import rclpy
        from controller_manager_msgs.srv import ListControllers
        from geometry_msgs.msg import Twist
        from rclpy.parameter import Parameter
        from sensor_msgs.msg import JointState

        self.rclpy, self.Twist = rclpy, Twist
        self.spec, self.urdf_text = spec, urdf_text
        rclpy.init()
        self.node = rclpy.create_node("sim_benchmark", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.joints = {}
        self.node.create_subscription(JointState, "/joint_states", lambda m: self.joints.update(zip(m.name, m.position)), 50)
        self.cmd = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.list_controllers = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
        self.ListControllers = ListControllers

    def cmd_vel_guard(self):
        """Refuse to publish /cmd_vel directly when the graph runs the approved command chain (see the module docstring)."""
        names = [f"{ns.rstrip('/')}/{n}" for n, ns in self.node.get_node_names_and_namespaces()]
        reason = bm.direct_cmd_vel_refusal(names)
        if reason:
            raise RuntimeError(reason)

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def spin_until(self, predicate, wall_s):
        end = time.monotonic() + wall_s
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return predicate()

    def spin_sim(self, seconds, on_tick=None):
        start = self.now()
        while self.now() - start < seconds:
            if on_tick:
                on_tick(self.now() - start)
            self.rclpy.spin_once(self.node, timeout_sec=0.02)

    def wait_ready(self, wall_s=300):
        if not self.list_controllers.wait_for_service(timeout_sec=wall_s):
            raise RuntimeError("no /controller_manager/list_controllers")
        end = time.monotonic() + wall_s
        while time.monotonic() < end:
            fut = self.list_controllers.call_async(self.ListControllers.Request())
            self.spin_until(fut.done, 5.0)
            states = {c.name: c.state for c in fut.result().controller} if fut.done() and fut.result() else {}
            if all(states.get(c) == "active" for c in CONTROLLERS) and all(j in self.joints for j in LEFT + ["lift_joint"]) \
                    and self.now() > 0:
                return states
            time.sleep(0.5)
        raise RuntimeError(f"controllers not active: {states}")

    def trajectory(self, controller, names, points):
        """points: [(time_s, positions, velocities)]. Blocks until the result; returns the error code."""
        from builtin_interfaces.msg import Duration
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        from trajectory_msgs.msg import JointTrajectoryPoint
        client = ActionClient(self.node, FollowJointTrajectory, f"/{controller}/follow_joint_trajectory")
        if not client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError(f"{controller} has no action server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        for t, pos, vel in points:
            sec = int(t)
            goal.trajectory.points.append(JointTrajectoryPoint(positions=[float(x) for x in pos], velocities=[float(v) for v in vel],
                                                               time_from_start=Duration(sec=sec, nanosec=int(round((t - sec) * 1e9)))))
        send = client.send_goal_async(goal)
        if not self.spin_until(send.done, 30.0) or not send.result().accepted:
            raise RuntimeError(f"{controller} did not accept the goal")
        result = send.result().get_result_async()
        if not self.spin_until(result.done, 900.0):
            raise RuntimeError(f"{controller} trajectory did not finish")
        return result.result().result.error_code

    def q(self, name):
        self.spin_until(lambda: False, 0.1)
        return float(self.joints[name])


def run(spec, urdf_text, sampler):
    b = Bench(spec, urdf_text)
    sc = spec["scenarios"]
    meta = {"controllers": b.wait_ready()}
    b.cmd_vel_guard()     # after the controllers are up, so the launch's nodes are discovered; raises before any command
    results = {}
    b.spin_sim(spec["settle_s"])
    _, poses = read_poses()
    R, p = base_pose(poses)
    pitch, roll = bm.pitch_roll_of(R)
    results["rest"] = {"base_height_m": p[2], "base_pitch_deg": pitch, "base_roll_deg": roll}

    s = sc["joint_step"]
    start = {j: b.q(j) for j in LEFT}
    target = dict(start, **{s["joint"]: s["target"]})
    _, p0 = base_pose(read_poses()[1])
    meta["joint_step_error_code"] = b.trajectory("left_arm_controller", LEFT, [(s["move_s"], [target[j] for j in LEFT], [0.0] * 7)])
    executions = {}
    hold = []
    b.spin_sim(s["hold_s"], lambda t: hold.append(abs(b.joints[s["joint"]] - s["target"])))
    _, p1 = base_pose(read_poses()[1])
    results["joint_step"] = {"final_error_rad": abs(b.q(s["joint"]) - s["target"]), "max_hold_error_rad": max(hold),
                             "base_displacement_m": math.hypot(p1[0] - p0[0], p1[1] - p0[1])}
    executions["joint_step"] = bm.execution(abs(s["target"] - start[s["joint"]]), {"joint_states": abs(b.q(s["joint"]) - start[s["joint"]])},
                                            meta["joint_step_error_code"] == SUCCESSFUL, s["execution"])

    s = sc["arm_reach"]
    reach0 = {j: b.q(j) for j in LEFT}
    meta["arm_reach_error_code"] = b.trajectory("left_arm_controller", LEFT,
                                                [(s["move_s"], [s["targets"][j] for j in LEFT], [0.0] * 7)])
    b.spin_sim(s["hold_s"])
    _, poses = read_poses()
    ee = link_in_base(poses, s["link"])
    results["arm_reach"] = {"ee_x_m": ee[0], "ee_y_m": ee[1], "ee_z_m": ee[2],
                            "max_joint_error_rad": max(abs(b.q(j) - t) for j, t in s["targets"].items())}
    widest = max(s["targets"], key=lambda j: abs(s["targets"][j] - reach0[j]))
    executions["arm_reach"] = bm.execution(abs(s["targets"][widest] - reach0[widest]), {"joint_states": abs(b.q(widest) - reach0[widest])},
                                           meta["arm_reach_error_code"] == SUCCESSFUL, s["execution"])

    s = sc["lift_move"]
    import re
    vel = float(re.search(r'<joint name="lift_joint"[\s\S]*?velocity="([0-9.eE+-]+)"', urdf_text).group(1))
    ramp_s = abs(s["to"] - s["from"]) / vel
    times = [min(ramp_s, 0.5 * k) for k in range(1, int(math.ceil(ramp_s / 0.5)) + 1)]
    points = [(t, [bm.lift_ramp(s["from"], s["to"], vel, t)], [vel if t < ramp_s else 0.0]) for t in times]
    poses0 = read_poses()[1]
    R0, _ = base_pose(poses0)
    lift0 = b.q("lift_joint")
    carriage0 = link_in_base(poses0, CARRIAGE)[2] if CARRIAGE in poses0 else None
    track = {"max": 0.0}
    t_start = b.now()

    def lift_track():
        t = b.now() - t_start
        track["max"] = max(track["max"], abs(bm.lift_ramp(s["from"], s["to"], vel, t) - b.joints["lift_joint"]))
    timer = b.node.create_timer(0.02, lift_track)
    meta["lift_error_code"] = b.trajectory("lift_controller", ["lift_joint"], points)
    b.spin_sim(s["hold_s"])
    b.node.destroy_timer(timer)
    poses1 = read_poses()[1]
    R1, _ = base_pose(poses1)
    lift1 = b.q("lift_joint")
    carriage1 = link_in_base(poses1, CARRIAGE)[2] if CARRIAGE in poses1 else None
    results["lift_move"] = {"final_position_m": lift1, "max_tracking_error_m": track["max"],
                            "base_pitch_change_deg": bm.pitch_roll_of(R1)[0] - bm.pitch_roll_of(R0)[0], "ramp_velocity_m_s": vel}
    meta["lift_carriage_ground_truth_m"] = [carriage0, carriage1]
    executions["lift_move"] = bm.execution(
        abs(s["to"] - lift0),
        {"joint_states": abs(lift1 - lift0),
         "gazebo_link_pose": None if carriage0 is None or carriage1 is None else abs(carriage1 - carriage0)},
        meta["lift_error_code"] == SUCCESSFUL, s["execution"])

    dd = bm.diffdrive_params(urdf_text)
    for name, lin, ang, on_key in (("straight_drive", sc["straight_drive"]["linear"], 0.0, "drive_s"),
                                   ("in_place_turn", 0.0, sc["in_place_turn"]["angular"], "turn_s")):
        s = sc[name]
        _, ang_int = bm.shaped_profile(ang, s[on_key], s["stop_s"], dd["max_angular_velocity"], dd["max_angular_acceleration"],
                                       dd["min_angular_acceleration"], 1.0 / 750.0)
        t0, poses = read_poses()
        R0, p0 = base_pose(poses)
        twist = b.Twist()
        twist.linear.x, twist.angular.z = float(lin), float(ang)
        b.spin_sim(s[on_key], lambda t, tw=twist: b.cmd.publish(tw))
        b.spin_sim(s["stop_s"], lambda t: b.cmd.publish(b.Twist()))
        t1, poses = read_poses()
        R1, p1 = base_pose(poses)
        mid = [bm.yaw_of(base_pose(pp)[0]) for _, pp in sampler.between(t0, t1)]
        yaws = [bm.yaw_of(R0)] + mid + [bm.yaw_of(R1)]
        times = [t0] + [t for t, _ in sampler.between(t0, t1)] + [t1]
        gaps = [b2 - a for a, b2 in zip(times, times[1:])]
        mo = bm.planar_motion(p0, yaws[0], p1, yaws)
        meta[f"{name}_pose_samples"] = len(yaws)
        meta[f"{name}_max_sample_gap_s"] = max(gaps) if gaps else None
        if name == "straight_drive":
            results[name] = {"distance_m": mo["along_m"], "lateral_drift_m": mo["lateral_m"], "yaw_change_rad": mo["yaw_change_rad"]}
        else:
            # samples further apart than pi / omega cannot be unwrapped: then the turn result is not a result
            if meta[f"{name}_max_sample_gap_s"] is not None and meta[f"{name}_max_sample_gap_s"] * s["angular"] >= math.pi:
                meta["in_place_turn_blocked"] = "pose samples too far apart to unwrap the yaw"
            else:
                results[name] = {"yaw_over_commanded": mo["yaw_change_rad"] / ang_int, "centre_drift_m": mo["displacement_m"],
                                 "commanded_yaw_rad": ang_int}

    s = sc["object_contact"]
    h = s["size"] / 2.0
    i = s["mass"] * (2 * s["size"] ** 2) / 12.0
    sdf = (f'<sdf version="1.9"><model name="{BOX}"><link name="box"><inertial><mass>{s["mass"]}</mass><inertia><ixx>{i}</ixx>'
           f'<iyy>{i}</iyy><izz>{i}</izz></inertia></inertial><collision name="c"><geometry><box><size>{s["size"]} {s["size"]} '
           f'{s["size"]}</size></box></geometry></collision><visual name="v"><geometry><box><size>{s["size"]} {s["size"]} '
           f'{s["size"]}</size></box></geometry></visual></link></model></sdf>')
    x, y, z = s["position"]
    subprocess.run(["ros2", "run", "ros_gz_sim", "create", "-world", WORLD, "-name", BOX, "-string", sdf,
                    "-x", str(x), "-y", str(y), "-z", str(z)], check=True, timeout=120)
    release = None
    end = time.monotonic() + 60
    while release is None and time.monotonic() < end:
        tr, poses = read_poses()
        if BOX in poses:
            release = poses[BOX][0]
    if release is None:
        raise RuntimeError("the box never appeared on the pose topic")
    b.spin_sim(s["settle_s"])
    ta, pa = read_poses()
    b.spin_sim(0.2)
    tb, pb = read_poses()
    za, zb = pa[BOX][0], pb[BOX][0]
    speed = math.dist(za, zb) / (tb - ta) if tb > ta else None
    results["object_contact"] = {"rest_height_m": zb[2], "horizontal_slip_m": math.hypot(zb[0] - release[0], zb[1] - release[1])}
    if speed is not None:
        results["object_contact"]["final_speed_m_s"] = speed
    b.node.destroy_node()
    b.rclpy.shutdown()
    return results, meta, executions


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", required=True)
    parser.add_argument("--launch-log", default="gazebo_benchmark_launch.log")
    opts = parser.parse_args()
    spec = bm.load_scenarios()
    out = pathlib.Path(opts.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    log = open(opts.launch_log, "w")
    launch = subprocess.Popen(["ros2", "launch", "openamrobot_upperbody_gazebo", "sim.launch.py", "headless:=true",
                               f"base_plate:={spec['configuration']}"], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    doc = {"schema": bm.SCHEMA, "backend": "gazebo", "configuration": spec["configuration"], "start_keyframe": spec["start_keyframe"],
           "evidence_class": bm.EVIDENCE_CLASS, "base_command_path": "/cmd_vel direct to the DiffDrive plugin (no command chain)",
           "results": {}, "execution": {}, "meta": {}}
    sampler = PoseSampler()
    rc = 1
    try:
        desc = subprocess.run(["ros2", "param", "get", "/robot_state_publisher", "robot_description", "--hide-type"],
                              capture_output=True, text=True, timeout=300)
        tries = 0
        while ("<robot" not in desc.stdout) and tries < 30:
            time.sleep(5)
            tries += 1
            desc = subprocess.run(["ros2", "param", "get", "/robot_state_publisher", "robot_description", "--hide-type"],
                                  capture_output=True, text=True, timeout=120)
        urdf_text = desc.stdout
        if "<robot" not in urdf_text:
            raise RuntimeError("robot_description not available")
        import hashlib
        doc["meta"]["robot_description_bytes"] = len(urdf_text.encode())
        doc["meta"]["robot_description_sha256_unnormalised"] = hashlib.sha256(urdf_text.encode()).hexdigest()
        doc["diffdrive"] = bm.diffdrive_params(urdf_text)
        sampler.start()
        results, meta, executions = run(spec, urdf_text, sampler)
        doc["results"] = {k: {mk: float(mv) for mk, mv in v.items()} for k, v in results.items()}
        doc["execution"] = executions
        doc["meta"].update(meta)
        doc["meta"]["pose_sampler_errors"] = sampler.errors
        rc = 0
    except Exception as err:  # noqa: BLE001  (the document records the failure; missing results read as BLOCKED)
        doc["meta"]["error"] = f"{type(err).__name__}: {err}"
    finally:
        sampler.stop = True
        try:
            os.killpg(launch.pid, signal.SIGINT)
            launch.wait(timeout=60)
        except Exception:  # noqa: BLE001
            os.killpg(launch.pid, signal.SIGKILL)
        out.write_text(json.dumps(doc, indent=1, default=str) + "\n")
        print(json.dumps(doc, indent=1, default=str))
    return rc


if __name__ == "__main__":
    sys.exit(main())
