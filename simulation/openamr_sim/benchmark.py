# SPDX-License-Identifier: MIT
"""Backend benchmark: the scenarios of simulation/benchmark/scenarios.yaml, their command shaping, their metrics, the MuJoCo runner
and the comparison of backends.

One definition of each command and each metric serves every backend: the Gazebo runner (simulation/benchmark/gazebo_benchmark.py)
and the Isaac runner (simulation/isaac/isaac_scenarios.py) import the shaping and metric functions from here, so a metric cannot
mean one thing in one backend and another thing in the other. The MuJoCo runner is below.

Nothing here tunes a backend. A difference beyond tolerance is a finding to explain, never a parameter to change.
Simulation evidence only: not hardware, physical acceptance or safety evidence.
"""
import math
import pathlib
import re
import xml.etree.ElementTree as ET

import yaml

from . import arm_gravity

SCENARIOS = pathlib.Path(__file__).resolve().parents[1] / "benchmark" / "scenarios.yaml"
ORDER = ("rest", "joint_step", "arm_reach", "lift_move", "straight_drive", "in_place_turn", "object_contact")
SCHEMA = "openamr.sim_benchmark.v1"


def load_scenarios(path=SCENARIOS):
    spec = yaml.safe_load(pathlib.Path(path).read_text())
    if tuple(spec["scenarios"]) != ORDER:
        raise ValueError(f"scenario order {tuple(spec['scenarios'])} is not {ORDER}")
    return spec


# ------------------------------------------------------------------------------------------------------------ command shaping
def diffdrive_params(urdf_text):
    """The DiffDrive plugin's geometry and limits, read from the expansion (the values Gazebo runs with)."""
    m = re.search(r'<plugin[^>]*DiffDrive[^>]*>([\s\S]*?)</plugin>', urdf_text)
    if m is None:
        raise ValueError("no DiffDrive plugin in the expansion")
    body = m.group(1)

    def num(tag):
        hit = re.search(rf"<{tag}>\s*([-0-9.eE+]+)\s*</{tag}>", body)
        if hit is None:
            raise ValueError(f"DiffDrive has no <{tag}>")
        return float(hit.group(1))
    return {"wheel_separation": num("wheel_separation"), "wheel_radius": num("wheel_radius"),
            "max_linear_velocity": num("max_linear_velocity"), "max_linear_acceleration": num("max_linear_acceleration"),
            "min_linear_acceleration": num("min_linear_acceleration"), "max_angular_velocity": num("max_angular_velocity"),
            "max_angular_acceleration": num("max_angular_acceleration"), "min_angular_acceleration": num("min_angular_acceleration")}


class Limiter:
    """Velocity and acceleration limits applied to a commanded value at each update, as the DiffDrive plugin's speed limiter does."""

    def __init__(self, vmax, accel_up, accel_down):
        self.vmax, self.up, self.down, self.value = vmax, accel_up, abs(accel_down), 0.0

    def step(self, command, dt):
        target = max(-self.vmax, min(self.vmax, command))
        delta = target - self.value
        speeding_up = abs(target) > abs(self.value) and target * self.value >= 0
        limit = (self.up if speeding_up else self.down) * dt
        self.value += max(-limit, min(limit, delta))
        return self.value


def shaped_profile(command, on_s, off_s, vmax, accel_up, accel_down, dt):
    """The limited command at every dt: `command` for on_s, then 0 for off_s. Returns the list and its integral."""
    lim = Limiter(vmax, accel_up, accel_down)
    values = [lim.step(command if k * dt < on_s else 0.0, dt) for k in range(int(round((on_s + off_s) / dt)))]
    return values, sum(values) * dt


def wheel_speeds(linear, angular, separation, radius):
    """(left, right) wheel angular speeds in rad/s, the DiffDrive plugin's kinematics."""
    return (linear - angular * separation / 2.0) / radius, (linear + angular * separation / 2.0) / radius


def cubic(q0, q1, duration, t):
    """Cubic with zero end velocities: the interpolation of a JointTrajectoryController given one point with zero velocity."""
    s = min(max(t / duration, 0.0), 1.0)
    return q0 + (q1 - q0) * (3 * s * s - 2 * s * s * s)


def lift_ramp(start, end, velocity, t):
    return min(end, start + velocity * t) if end >= start else max(end, start - velocity * t)


# ---------------------------------------------------------------------------------------------------------- execution check
def execution(commanded_travel, measured_travel, controller_success, rule):
    """Whether a commanded joint move was executed, judged from MEASURED motion only.

    commanded_travel: |target - start| of the move. measured_travel: {source: |end - start|} from each measurement the backend
    has (Gazebo: /joint_states and the simulator's own link pose; MuJoCo: qpos); a source that could not be read is None.
    controller_success: what the controller reported (True, False, or None where no controller is involved). It is recorded and
    never decides the verdict: a JointTrajectoryController without goal tolerances reports success when the trajectory time has
    passed, whether the joint moved or not.

    PASS when every measured source travelled at least rule["min_travel_fraction"] of the commanded travel; FAIL when any source
    travelled less; BLOCKED when no source could be read."""
    fraction = float(rule["min_travel_fraction"])
    read = {k: float(v) for k, v in measured_travel.items() if v is not None}
    doc = {"commanded_travel": float(commanded_travel), "measured_travel": {k: (None if v is None else float(v))
                                                                            for k, v in measured_travel.items()},
           "min_travel_fraction": fraction, "controller_reported_success": controller_success}
    if not read:
        doc.update(verdict="BLOCKED", reason="no measurement of the joint could be read")
    elif all(v >= fraction * abs(commanded_travel) for v in read.values()):
        doc.update(verdict="PASS", reason="the measured joint moved")
    else:
        short = ", ".join(f"{k} {v:.4g}" for k, v in read.items() if v < fraction * abs(commanded_travel))
        why = "success without motion: the controller reported success" if controller_success else "no measured motion"
        doc.update(verdict="FAIL", reason=f"{why}; commanded travel {abs(commanded_travel):.4g}, measured {short}")
    return doc


# --------------------------------------------------------------------------------------------- direct base command guard
# Evidence class of every result document of this benchmark. The base scenarios publish geometry_msgs/Twist straight to /cmd_vel,
# the DiffDrive plugin's input. That isolates the physics of the base from the stack and is the only use this benchmark makes of it.
# It is never whole-robot acceptance: the robot's approved command chain (twist_mux, the stop gate, the velocity smoother, the
# collision monitor) is not in the loop, and a whole-robot run must command the base through that chain.
EVIDENCE_CLASS = "isolated physics benchmark (direct /cmd_vel to the DiffDrive plugin); not whole-robot acceptance"
COMMAND_CHAIN_NODES = ("twist_mux", "velocity_smoother", "collision_monitor", "controller_server", "bt_navigator")


def direct_cmd_vel_refusal(node_names):
    """None when the graph holds no node of the approved command chain, so a direct /cmd_vel publisher bypasses nothing; otherwise
    the reason to refuse. The benchmark runs only against the bare simulation (sim.launch.py), never against a whole-robot stack."""
    found = sorted({n for n in node_names if n.rsplit("/", 1)[-1] in COMMAND_CHAIN_NODES or "stop_gate" in n})
    if found:
        return ("refused: the graph runs the command chain (" + ", ".join(found) + "); this benchmark publishes /cmd_vel directly "
                "and is an isolated physics benchmark only. Whole-robot runs command the base through the approved chain.")
    return None


# ---------------------------------------------------------------------------------------------------------------- metrics
def yaw_of(R):
    return math.atan2(R[1][0], R[0][0])


def pitch_roll_of(R):
    return math.degrees(math.asin(max(-1.0, min(1.0, -R[2][0])))), math.degrees(math.atan2(R[2][1], R[2][2]))


def unwrap(angles):
    """Angles in rad made continuous: each step is taken as the shortest turn (samples must be less than pi apart)."""
    out = [angles[0]]
    for a in angles[1:]:
        out.append(out[-1] + (a - out[-1] + math.pi) % (2 * math.pi) - math.pi)
    return out


def planar_motion(p0, yaw0, p1, yaws):
    """Distance along the start heading, lateral drift and the unwrapped yaw change between two base poses."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    hx, hy = math.cos(yaw0), math.sin(yaw0)
    unwrapped = unwrap(list(yaws))
    return {"along_m": dx * hx + dy * hy, "lateral_m": -dx * hy + dy * hx, "yaw_change_rad": unwrapped[-1] - unwrapped[0],
            "displacement_m": math.hypot(dx, dy)}


def in_base(R_base, p_base, p):
    d = [p[i] - p_base[i] for i in range(3)]
    return [sum(R_base[r][i] * d[r] for r in range(3)) for i in range(3)]


# ------------------------------------------------------------------------------------------------------------ MuJoCo runner
def _with_box(xml_text, box):
    """The generated MJCF with the benchmark box as a free body and without keyframes (their qpos length would no longer fit)."""
    from .mjcf import CONAFFINITY_ROBOT, CONTYPE_ROBOT
    root = ET.fromstring(xml_text)
    for key in root.findall("keyframe"):
        root.remove(key)
    world = root.find("worldbody")
    h = box["size"] / 2.0
    body = ET.SubElement(world, "body", name="bench_box", pos=" ".join(str(v) for v in box["position"]))
    ET.SubElement(body, "freejoint", name="bench_box_free")
    i = box["mass"] * (2 * box["size"] ** 2) / 12.0  # solid box
    ET.SubElement(body, "inertial", pos="0 0 0", mass=str(box["mass"]), diaginertia=f"{i} {i} {i}")
    ET.SubElement(body, "geom", name="bench_box_geom", type="box", size=f"{h} {h} {h}", mass="0",
                  contype=str(CONTYPE_ROBOT), conaffinity=str(CONAFFINITY_ROBOT))
    return ET.tostring(root, encoding="unicode")


# Drive wheel contact shapes of platform-sw pull request 171 (sim/wheel-contact-option, head 747cf28, robo_urdf.urdf.xacro
# macro drive_wheel_collision; Proposed, awaiting the robot-description owner and the software lead). A diagnostic of the
# benchmark only: the default "pinned" runs the generated model unchanged. Each other value replaces only the two drive wheel
# collision geoms of the generated MJCF and the DiffDrive radius used to shape the command, as pull request 171 (on 146, the
# loaded radius 0.098) sets them. Nothing canonical changes here; the shape is decided in platform-sw.
WHEEL_CONTACT = {
    "pinned": None,
    "cylinder": {"type": "cylinder", "size": "0.098 0.023", "axis_y": True, "radius": 0.098},
    "narrow_cylinder": {"type": "cylinder", "size": "0.098 0.002", "axis_y": True, "radius": 0.098},
    "sphere": {"type": "sphere", "size": "0.098", "axis_y": False, "radius": 0.098},
}
DRIVE_WHEEL_GEOMS = ("left_wheel_link__collision_0", "right_wheel_link__collision_0")


def _with_wheel_contact(root, option):
    """Replace the drive wheel collision geoms of a parsed MJCF by the pull request 171 shape `option` (in place)."""
    shape = WHEEL_CONTACT[option]
    if shape is None:
        return
    found = 0
    for geom in root.iter("geom"):
        if geom.get("name") in DRIVE_WHEEL_GEOMS:
            geom.set("type", shape["type"])
            geom.set("size", shape["size"])
            geom.set("quat", "0.707106790659974 0.707106771713121 0 0" if shape["axis_y"] else "1 0 0 0")
            found += 1
    if found != len(DRIVE_WHEEL_GEOMS):
        raise ValueError(f"expected {len(DRIVE_WHEEL_GEOMS)} drive wheel collision geoms, found {found}")


def run_mujoco(mjcf_path, urdf_text, overrides, spec=None, arm_gravity_mode=None, wheel_contact="pinned"):
    """Run every scenario in order in one MuJoCo simulation. Returns the result document (schema openamr.sim_benchmark.v1).

    arm_gravity_mode: None takes engine_overrides.yaml actuators.arm_gravity_feedforward (default build1, the real driver path of
    integration/baseline1); "off" runs the MIT gains with tau 0 (main's real profile). wheel_contact: see WHEEL_CONTACT."""
    import mujoco as mj
    import numpy as np

    from .mjcf import FREE_JOINT
    from .physics import controller_period_steps

    spec = spec or load_scenarios()
    sc = spec["scenarios"]
    path = pathlib.Path(mjcf_path)
    base_model = mj.MjModel.from_xml_path(str(path))
    key = base_model.key(spec["start_keyframe"]).id
    start_qpos = base_model.key_qpos[key].copy()
    xml = _with_box(path.read_text(), sc["object_contact"])
    if wheel_contact != "pinned":
        tree = ET.fromstring(xml)
        _with_wheel_contact(tree, wheel_contact)
        xml = ET.tostring(tree, encoding="unicode")
    cwd = pathlib.Path.cwd()
    try:  # mesh paths in the MJCF are relative to its directory
        import os
        os.chdir(path.parent)
        m = mj.MjModel.from_xml_string(xml)
    finally:
        os.chdir(cwd)
    d = mj.MjData(m)
    d.qpos[:len(start_qpos)] = start_qpos
    box_adr = m.jnt_qposadr[m.joint("bench_box_free").id]
    box_dof = m.jnt_dofadr[m.joint("bench_box_free").id]
    box_home = np.array(list(sc["object_contact"]["position"]) + [1, 0, 0, 0], dtype=float)
    d.qpos[box_adr:box_adr + 7] = box_home
    period = controller_period_steps(m, overrides)
    dt = m.opt.timestep
    ctrl_dt = period * dt
    pos_act = {m.actuator(i).name[:-len("_position")]: i for i in range(m.nu) if m.actuator(i).name.endswith("_position")}
    mj.mj_forward(m, d)
    for jn, a in pos_act.items():
        d.ctrl[a] = d.qpos[m.jnt_qposadr[m.joint(jn).id]]
    dd = diffdrive_params(urdf_text)
    if WHEEL_CONTACT[wheel_contact] is not None:
        dd["wheel_radius"] = WHEEL_CONTACT[wheel_contact]["radius"]
    grav = arm_gravity.settings(overrides)
    if arm_gravity_mode is not None:
        if arm_gravity_mode not in arm_gravity.MODES:
            raise ValueError(f"arm_gravity_mode {arm_gravity_mode!r} is not one of {arm_gravity.MODES}")
        grav["mode"] = arm_gravity_mode
    ff = arm_gravity.GravityFeedForward(m, grav["max_state_age_s"]) if grav["mode"] == "build1" else None
    ff_period = max(1, int(round(1.0 / (grav["rate_hz"] * dt))))
    arm_dofs = [m.jnt_dofadr[m.joint(jn).id] for jn in arm_gravity.ARM_JOINTS["left"]]
    arm_acts = [pos_act[jn] for jn in arm_gravity.ARM_JOINTS["left"]]
    arm_effort = [float(m.actuator_forcerange[a][1]) for a in arm_acts]
    usage = {"max": 0.0}
    wl, wr = m.actuator("left_wheel_joint_velocity").id, m.actuator("right_wheel_joint_velocity").id
    jwl, jwr = m.joint("left_wheel_joint").id, m.joint("right_wheel_joint").id
    state = {"box_free": False, "k": 0}

    def q(jn):
        return float(d.qpos[m.jnt_qposadr[m.joint(jn).id]])

    def base():
        b = d.body("base_footprint")
        return b.xpos.copy(), b.xmat.reshape(3, 3).copy()

    def step(seconds, on_ctrl=None, on_step=None):
        n = int(round(seconds / dt))
        for i in range(n):
            if state["k"] % period == 0 and on_ctrl is not None:
                on_ctrl(i * dt)
            if state["k"] % ff_period == 0:
                if ff is not None:
                    ff.on_state(d, d.time)
                    ff.apply(d, ff.command(d.time)[0])
                usage["max"] = max(usage["max"], max(abs(d.actuator_force[a] + d.qfrc_applied[j]) / e
                                                     for a, j, e in zip(arm_acts, arm_dofs, arm_effort)))
            if not state["box_free"]:
                d.qpos[box_adr:box_adr + 7] = box_home
                d.qvel[box_dof:box_dof + 6] = 0
            mj.mj_step(m, d)
            state["k"] += 1
            if on_step is not None:
                on_step(i * dt)

    results = {}
    step(spec["settle_s"])
    p, R = base()
    pitch, roll = pitch_roll_of(R.tolist())
    results["rest"] = {"base_height_m": float(p[2]), "base_pitch_deg": pitch, "base_roll_deg": roll}

    executions = {}
    s = sc["joint_step"]
    jn, q0 = s["joint"], q(s["joint"])
    p0, _ = base()

    def js_ctrl(t):
        d.ctrl[pos_act[jn]] = cubic(q0, s["target"], s["move_s"], t)
    step(s["move_s"], js_ctrl)
    hold = []
    step(s["hold_s"], lambda t: None, lambda t: hold.append(abs(q(jn) - s["target"])))
    p1, _ = base()
    results["joint_step"] = {"final_error_rad": abs(q(jn) - s["target"]), "max_hold_error_rad": max(hold),
                             "base_displacement_m": float(np.linalg.norm((p1 - p0)[:2]))}
    executions["joint_step"] = execution(abs(s["target"] - q0), {"qpos": abs(q(jn) - q0)}, None, s["execution"])

    s = sc["arm_reach"]
    start = {j: q(j) for j in s["targets"]}

    def reach_ctrl(t):
        for j, target in s["targets"].items():
            d.ctrl[pos_act[j]] = cubic(start[j], target, s["move_s"], t)
    step(s["move_s"], reach_ctrl)
    step(s["hold_s"])
    pb, Rb = base()
    ee = in_base(Rb.tolist(), pb.tolist(), d.body(s["link"]).xpos.tolist())
    results["arm_reach"] = {"ee_x_m": ee[0], "ee_y_m": ee[1], "ee_z_m": ee[2],
                            "max_joint_error_rad": max(abs(q(j) - t) for j, t in s["targets"].items())}
    widest = max(s["targets"], key=lambda j: abs(s["targets"][j] - start[j]))
    executions["arm_reach"] = execution(abs(s["targets"][widest] - start[widest]), {"qpos": abs(q(widest) - start[widest])}, None,
                                        s["execution"])

    s = sc["lift_move"]
    import re as _re
    vel = float(_re.search(r'<joint name="lift_joint"[\s\S]*?velocity="([0-9.eE+-]+)"', urdf_text).group(1))
    ramp_s = abs(s["to"] - s["from"]) / vel
    lift0 = q("lift_joint")
    _, R0 = base()
    pitch0, _ = pitch_roll_of(R0.tolist())
    track = {"max": 0.0, "target": s["from"]}

    def lift_ctrl(t):
        track["target"] = lift_ramp(s["from"], s["to"], vel, t)
        d.ctrl[pos_act["lift_joint"]] = track["target"]
    step(ramp_s + s["hold_s"], lift_ctrl, lambda t: track.__setitem__("max", max(track["max"], abs(track["target"] - q("lift_joint")))))
    _, R1 = base()
    pitch1, _ = pitch_roll_of(R1.tolist())
    results["lift_move"] = {"final_position_m": q("lift_joint"), "max_tracking_error_m": track["max"],
                            "base_pitch_change_deg": pitch1 - pitch0, "ramp_velocity_m_s": vel}
    executions["lift_move"] = execution(abs(s["to"] - lift0), {"qpos": abs(q("lift_joint") - lift0)}, None, s["execution"])

    for name, lin, ang, on_key in (("straight_drive", sc["straight_drive"]["linear"], 0.0, "drive_s"),
                                   ("in_place_turn", 0.0, sc["in_place_turn"]["angular"], "turn_s")):
        s = sc[name]
        lin_prof, _ = shaped_profile(lin, s[on_key], s["stop_s"], dd["max_linear_velocity"], dd["max_linear_acceleration"],
                                     dd["min_linear_acceleration"], ctrl_dt)
        ang_prof, ang_int = shaped_profile(ang, s[on_key], s["stop_s"], dd["max_angular_velocity"], dd["max_angular_acceleration"],
                                           dd["min_angular_acceleration"], ctrl_dt)
        p0, R0 = base()
        yaws = [yaw_of(R0.tolist())]
        wheel = {"speed": [], "torque": []}

        def drive_ctrl(t, lp=lin_prof, ap=ang_prof):
            i = min(int(round(t / ctrl_dt)), len(lp) - 1)
            left, right = wheel_speeds(lp[i], ap[i], dd["wheel_separation"], dd["wheel_radius"])
            d.ctrl[wl], d.ctrl[wr] = left, right

        def sample(t, on_s=s[on_key]):
            if int(round(t / dt)) % period == 0:
                yaws.append(yaw_of(d.body("base_footprint").xmat.reshape(3, 3).tolist()))
                if 2.0 <= t < on_s and d.ctrl[wr] != 0.0:  # steady command: past the acceleration ramp, before the stop
                    wheel["speed"].append(0.5 * (d.qvel[m.jnt_dofadr[jwl]] / d.ctrl[wl] + d.qvel[m.jnt_dofadr[jwr]] / d.ctrl[wr]))
                    wheel["torque"].append(0.5 * (abs(d.actuator_force[wl]) + abs(d.actuator_force[wr])))
        step(s[on_key] + s["stop_s"], drive_ctrl, sample)
        d.ctrl[wl] = d.ctrl[wr] = 0.0
        p1, R1 = base()
        yaws.append(yaw_of(R1.tolist()))
        mo = planar_motion(p0.tolist(), yaws[0], p1.tolist(), yaws)
        if name == "straight_drive":
            results[name] = {"distance_m": mo["along_m"], "lateral_drift_m": mo["lateral_m"], "yaw_change_rad": mo["yaw_change_rad"]}
        else:
            results[name] = {"yaw_over_commanded": mo["yaw_change_rad"] / ang_int, "centre_drift_m": mo["displacement_m"],
                             "commanded_yaw_rad": ang_int}
        # Where a commanded wheel speed goes: the wheel's own speed over its command and the servo torque, both averaged
        # over the steady part of the command (diagnostic; not compared, Gazebo does not report them).
        results[name]["wheel_speed_over_commanded"] = float(np.mean(wheel["speed"])) if wheel["speed"] else float("nan")
        results[name]["wheel_servo_torque_Nm"] = float(np.mean(wheel["torque"])) if wheel["torque"] else float("nan")

    s = sc["object_contact"]
    state["box_free"] = True
    release = d.qpos[box_adr:box_adr + 3].copy()
    step(s["settle_s"])
    bp = d.qpos[box_adr:box_adr + 3]
    results["object_contact"] = {"rest_height_m": float(bp[2]), "horizontal_slip_m": float(np.linalg.norm((bp - release)[:2])),
                                 "final_speed_m_s": float(np.linalg.norm(d.qvel[box_dof:box_dof + 3]))}
    return {"schema": SCHEMA, "backend": "mujoco", "mujoco_version": mj.__version__, "timestep_s": dt,
            "controller_period_s": ctrl_dt, "diffdrive": dd, "configuration": spec["configuration"],
            "start_keyframe": spec["start_keyframe"], "free_joint": FREE_JOINT, "evidence_class": EVIDENCE_CLASS,
            "arm_gravity_feedforward": {"mode": grav["mode"], "rate_hz": grav["rate_hz"], "max_state_age_s": grav["max_state_age_s"],
                                        "left_arm_max_effort_fraction": usage["max"]},
            "wheel_contact": wheel_contact,
            "results": {k: {mk: float(mv) for mk, mv in v.items()} for k, v in results.items()}, "execution": executions}


# ---------------------------------------------------------------------------------------------------------------- comparison
def compare(docs, spec=None, reference="gazebo"):
    """Rows of (scenario, metric, value per backend, difference to the reference, tolerance, verdict).

    Verdict per backend pair: PASS within tolerance, FAIL beyond it, BLOCKED when either backend has no value (not run).
    A scenario with an `execution` rule (a commanded joint move) is judged first on measured motion in BOTH backends: when either
    did not execute the move every row of that scenario is FAIL, whatever the values (two backends that both stood still agree,
    and that is no parity); when either document carries no execution record for it the rows are BLOCKED."""
    spec = spec or load_scenarios()
    rows = []
    others = [b for b in docs if b != reference]
    for scen in ORDER:
        ruled = "execution" in spec["scenarios"][scen]
        for metric, tol in spec["scenarios"][scen]["metrics"].items():
            ref = ((docs.get(reference) or {}).get("results") or {}).get(scen, {}).get(metric)
            row = {"scenario": scen, "metric": metric, "values": {}, "tolerance": tol, "verdicts": {}, "differences": {}, "notes": {}}
            row["values"][reference] = ref
            for b in others:
                val = ((docs.get(b) or {}).get("results") or {}).get(scen, {}).get(metric)
                row["values"][b] = val
                if ref is None or val is None:
                    row["verdicts"][b] = "BLOCKED"
                    continue
                if ruled:
                    ex = {k: ((docs.get(k) or {}).get("execution") or {}).get(scen) for k in (reference, b)}
                    failed = [k for k, e in ex.items() if e is not None and e.get("verdict") == "FAIL"]
                    missing = [k for k, e in ex.items() if e is None or e.get("verdict") not in ("PASS", "FAIL")]
                    if failed:
                        row["verdicts"][b] = "FAIL"
                        row["notes"][b] = "; ".join(f"not executed in {k}: {ex[k]['reason']}" for k in failed)
                        row["differences"][b] = val - ref
                        continue
                    if missing:
                        row["verdicts"][b] = "BLOCKED"
                        row["notes"][b] = "no measured execution record in " + ", ".join(missing)
                        continue
                diff = val - ref
                row["differences"][b] = diff
                limit = tol["tolerance_abs"] if "tolerance_abs" in tol else tol["tolerance_rel"] * abs(ref)
                row["verdicts"][b] = "PASS" if abs(diff) <= limit else "FAIL"
            rows.append(row)
    return rows


def table(rows, reference="gazebo"):
    backends = [reference] + sorted({b for r in rows for b in r["values"] if b != reference})
    head = "| Scenario | Metric | " + " | ".join(backends) + " | " + " | ".join(f"{b} - {reference}" for b in backends[1:]) + \
        " | Tolerance | " + " | ".join(f"{b} verdict" for b in backends[1:]) + " |"
    lines = [head, "|" + "---|" * (head.count("|") - 1)]

    def f(v):
        return "not run" if v is None else f"{v:.4f}"
    for r in rows:
        tol = r["tolerance"]
        t = f"{tol['tolerance_abs']}" if "tolerance_abs" in tol else f"{tol['tolerance_rel'] * 100:g} %"
        lines.append(f"| {r['scenario']} | {r['metric']} | " + " | ".join(f(r["values"].get(b)) for b in backends) + " | " +
                     " | ".join(f(r["differences"].get(b)) for b in backends[1:]) + f" | {t} | " +
                     " | ".join(r["verdicts"].get(b, "BLOCKED") + (" (not executed)" if "not executed" in
                                                                   r.get("notes", {}).get(b, "") else "")
                                for b in backends[1:]) + " |")
    return "\n".join(lines)


def execution_table(docs):
    """One row per backend and commanded joint move: commanded and measured travel, the controller's own report, the verdict."""
    lines = ["| Backend | Scenario | Commanded travel | Measured travel | Controller reported success | Execution | Reason |",
             "|---|---|---|---|---|---|---|"]
    for backend, doc in docs.items():
        for scen, ex in sorted(((doc or {}).get("execution") or {}).items(), key=lambda kv: ORDER.index(kv[0])):
            measured = ", ".join(f"{k} {'unread' if v is None else f'{v:.4f}'}" for k, v in ex["measured_travel"].items())
            ctrl = {True: "yes", False: "no", None: "no controller"}[ex.get("controller_reported_success")]
            lines.append(f"| {backend} | {scen} | {ex['commanded_travel']:.4f} | {measured} | {ctrl} | {ex['verdict']} | {ex['reason']} |")
    return "\n".join(lines)
