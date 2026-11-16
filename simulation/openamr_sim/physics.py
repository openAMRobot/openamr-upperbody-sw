# SPDX-License-Identifier: MIT
"""Physics checks of the MuJoCo model, each within MuJoCo alone.

Nothing here compares MuJoCo with another engine. Each check states its own expectation from the model's parameters (or from an
independent rigid-body implementation, pinocchio, for gravity torques) and measures MuJoCo against it. A pass is development
evidence about the MuJoCo model only. It is never hardware, physical acceptance or safety evidence.
"""
import math

import numpy as np

from .mjcf import FREE_JOINT, load_yaml
from .urdf_model import URDF


def controller_period_steps(model, overrides):
    rate = overrides["common"]["controller_rate"]["value"]
    steps = 1.0 / (rate * model.opt.timestep)
    n = int(round(steps))
    if abs(steps - n) > 1e-6:
        raise ValueError(f"controller rate {rate} Hz is not a whole number of physics steps of {model.opt.timestep} s")
    return n


# ---------------------------------------------------------------------------------------------------------------- pinocchio
class PinocchioGravity:
    """Gravity torques of the expanded URDF with pinocchio (fixed base at the identity unless a gravity direction is given)."""

    def __init__(self, urdf_text):
        import pinocchio as pin

        self.pin = pin
        self.model = pin.buildModelFromXML(urdf_text)
        self.data = self.model.createData()

    def q_vector(self, q):
        pin, model = self.pin, self.model
        vec = pin.neutral(model)
        for name, value in q.items():
            if not model.existJointName(name):
                continue
            j = model.joints[model.getJointId(name)]
            if j.nq == 2:  # continuous joint: cos, sin
                vec[j.idx_q:j.idx_q + 2] = [math.cos(value), math.sin(value)]
            else:
                vec[j.idx_q] = value
        return vec

    def torques(self, q, gravity=(0.0, 0.0, -9.81)):
        pin, model = self.pin, self.model
        model.gravity.linear = np.asarray(gravity, float)
        tau = pin.computeGeneralizedGravity(model, self.data, self.q_vector(q))
        out = {}
        for jid in range(1, model.njoints):
            j = model.joints[jid]
            out[model.names[jid]] = float(tau[j.idx_v])
        return out


# ---------------------------------------------------------------------------------------------------------------- helpers
def set_state(mj, m, d, q, base_pose=None):
    mj.mj_resetData(m, d)
    adr = m.jnt_qposadr[m.joint(FREE_JOINT).id]
    d.qpos[adr:adr + 7] = base_pose if base_pose is not None else [0, 0, 0, 1, 0, 0, 0]
    for name, value in q.items():
        try:
            jid = m.joint(name).id
        except KeyError:
            continue
        d.qpos[m.jnt_qposadr[jid]] = value


def mujoco_gravity(mj, m, d, q):
    """qfrc_bias at zero velocity, per joint name (the generalized gravity force)."""
    set_state(mj, m, d, q)
    d.qvel[:] = 0
    mj.mj_forward(m, d)
    return {m.joint(j).name: float(d.qfrc_bias[m.jnt_dofadr[j]]) for j in range(m.njnt) if m.joint(j).name != FREE_JOINT}


def contacts_by_geom(mj, m, d):
    """Normal force per robot geom touching the floor, and every contact pair."""
    floor = m.geom("floor").id
    forces, pairs = {}, []
    f6 = np.zeros(6)
    for i in range(d.ncon):
        c = d.contact[i]
        mj.mj_contactForce(m, d, i, f6)
        g1, g2 = m.geom(c.geom1).name, m.geom(c.geom2).name
        pairs.append((g1, g2))
        if c.geom1 == floor or c.geom2 == floor:
            other = g2 if c.geom1 == floor else g1
            forces[other] = forces.get(other, 0.0) + float(f6[0])
    return forces, pairs


def base_pitch_roll(d):
    R = d.body("base_footprint").xmat.reshape(3, 3)
    return math.degrees(math.asin(-R[2, 0])), math.degrees(math.atan2(R[2, 1], R[2, 2]))


def settle(mj, m, d, key, seconds):
    mj.mj_resetDataKeyframe(m, d, m.key(key).id)
    steps = int(round(seconds / m.opt.timestep))
    for _ in range(steps):
        mj.mj_step(m, d)
    return steps


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


# ---------------------------------------------------------------------------------------------------------------- checks
def check_base_contacts(mj, m, d, key, allowed_links, seconds=2.0):
    """Settle on a flat floor. Returns what touches the floor, the load per contact and the base attitude."""
    settle(mj, m, d, key, seconds)
    forces, pairs = contacts_by_geom(mj, m, d)
    weight = float(sum(m.body_mass)) * -m.opt.gravity[2]
    touching_links = sorted({m.body(m.geom_bodyid[m.geom(g).id]).name for g in forces})
    floor_pairs_only = all("floor" in p for p in pairs)
    pitch, roll = base_pitch_roll(d)
    return {
        "key": key, "seconds": seconds, "weight_N": weight, "normal_force_sum_N": sum(forces.values()),
        "normal_force_by_geom_N": {k: round(v, 3) for k, v in sorted(forces.items())}, "touching_links": touching_links,
        "unexpected_links": sorted(set(touching_links) - set(allowed_links)), "only_floor_contacts": floor_pairs_only,
        "base_height_m": float(d.body("base_footprint").xpos[2]), "base_pitch_deg": pitch, "base_roll_deg": roll,
        "max_abs_qvel": float(np.abs(d.qvel).max()),
    }


def check_castor_swivel(mj, m, d, key, prefixes, wheel_speed, seconds, start_swivel):
    """Drive the base straight with both wheel servos; return each castor's swivel angle and whether it is on the floor."""
    mj.mj_resetDataKeyframe(m, d, m.key(key).id)
    for p in prefixes:
        d.qpos[m.jnt_qposadr[m.joint(f"{p}_caster_joint").id]] = start_swivel
    mj.mj_forward(m, d)
    for side in ("left", "right"):
        d.ctrl[m.actuator(f"{side}_wheel_joint_velocity").id] = wheel_speed
    start = d.body("base_footprint").xpos.copy()
    touched = {p: 0 for p in prefixes}
    steps = int(round(seconds / m.opt.timestep))
    for k in range(steps):
        mj.mj_step(m, d)
        if k > steps // 2:
            forces, _ = contacts_by_geom(mj, m, d)
            for p in prefixes:
                if any(g.startswith(f"{p}_wheel_link__collision") for g in forces):
                    touched[p] += 1
    travel = d.body("base_footprint").xpos - start
    half = steps - steps // 2 - 1
    return {
        "key": key, "wheel_speed_rad_s": wheel_speed, "seconds": seconds, "start_swivel_rad": start_swivel,
        "travel_m": travel.tolist(),
        "swivel_rad": {p: wrap(float(d.qpos[m.jnt_qposadr[m.joint(f"{p}_caster_joint").id]])) for p in prefixes},
        "floor_contact_fraction": {p: touched[p] / half for p in prefixes},
    }


def check_lift_travel(mj, m, d, key, overrides, lower, upper, velocity, hold_seconds=2.0):
    """Command lift_joint along a ramp at its velocity limit, updated at the controller rate, then hold."""
    period = controller_period_steps(m, overrides)
    mj.mj_resetDataKeyframe(m, d, m.key(key).id)
    a = m.actuator("lift_joint_position").id
    jadr = m.jnt_qposadr[m.joint("lift_joint").id]
    ramp = (upper - lower) / velocity
    steps = int(round((ramp + hold_seconds) / m.opt.timestep))
    max_err, max_q, max_force, updates = 0.0, -1e9, 0.0, 0
    target = lower
    for k in range(steps):
        if k % period == 0:
            t = k * m.opt.timestep
            target = min(upper, lower + velocity * t)
            d.ctrl[a] = target
            updates += 1
        mj.mj_step(m, d)
        q = float(d.qpos[jadr])
        max_err = max(max_err, abs(target - q))
        max_q = max(max_q, q)
        max_force = max(max_force, abs(float(d.actuator_force[a])))
    pitch, roll = base_pitch_roll(d)
    base = d.body("base_footprint")
    mount = d.body("openarm_left_base_link")
    shoulder_in_base = base.xmat.reshape(3, 3).T @ (mount.xpos - base.xpos)
    return {
        "key": key, "from_m": lower, "to_m": upper, "velocity_m_s": velocity, "controller_updates": updates,
        "simulated_s": steps * m.opt.timestep, "final_m": float(d.qpos[jadr]), "final_error_m": upper - float(d.qpos[jadr]),
        "max_tracking_error_m": max_err, "max_position_m": max_q, "max_force_N": max_force,
        "force_limit_N": float(m.actuator_forcerange[a, 1]), "base_pitch_deg": pitch, "base_roll_deg": roll,
        "shoulder_height_in_base_m": float(shoulder_in_base[2]),
    }


def check_arm_hold(mj, m, d, key, overrides, urdf_text, seconds=3.0, gripper_fraction=0.5):
    """Hold both arms at the keyframe's targets with the controllers' position interface, written at the controller rate.

    The grippers are held at gripper_fraction of their range: the SRDF open and closed states both sit on a joint limit, where the
    limit, not the actuator, carries the fingers' weight, so a gravity balance there would not test the actuator.

    Returns per joint: the hold error, the steady actuator torque, pinocchio's gravity torque at the reached configuration (with
    gravity in the base frame, since the base rests slightly tilted) and the joint's frictionloss.
    """
    period = controller_period_steps(m, overrides)
    mj.mj_resetDataKeyframe(m, d, m.key(key).id)
    for side in ("left", "right"):
        for k in (1, 2):
            jid = m.joint(f"openarm_{side}_finger_joint{k}").id
            lo, hi = m.jnt_range[jid]
            d.qpos[m.jnt_qposadr[jid]] = lo + gripper_fraction * (hi - lo)
        a = m.actuator(f"openarm_{side}_finger_joint1_position").id
        lo, hi = m.jnt_range[m.joint(f"openarm_{side}_finger_joint1").id]
        d.ctrl[a] = lo + gripper_fraction * (hi - lo)
    mj.mj_forward(m, d)
    targets = d.ctrl.copy()
    updates = 0
    steps = int(round(seconds / m.opt.timestep))
    for k in range(steps):
        if k % period == 0:
            d.ctrl[:] = targets  # the trajectory controller holds its last point
            updates += 1
        mj.mj_step(m, d)
    u = URDF(urdf_text)
    q = {j.name: float(d.qpos[m.jnt_qposadr[m.joint(j.name).id]]) for j in u.movable()}
    R = d.body("base_footprint").xmat.reshape(3, 3)
    g_base = R.T @ m.opt.gravity
    tau_g = PinocchioGravity(urdf_text).torques(q, gravity=g_base)
    joints = {}
    for side in ("left", "right"):
        for name in [f"openarm_{side}_joint{k}" for k in range(1, 8)] + [f"openarm_{side}_finger_joint1"]:
            a = m.actuator(f"{name}_position").id
            dof = m.jnt_dofadr[m.joint(name).id]
            joints[name] = {
                "target": float(targets[a]), "position": q[name], "error": float(targets[a] - q[name]),
                "velocity": float(d.qvel[dof]), "actuator_torque": float(d.actuator_force[a]),
                "gravity_torque_pinocchio": tau_g[name], "frictionloss": float(m.dof_frictionloss[dof]),
                "force_limit": float(m.actuator_forcerange[a, 1]), "kp": float(m.actuator_gainprm[a, 0]),
            }
            follower = [f for f in u.joints.values() if f.mimic is not None and f.mimic["joint"] == name]
            if follower:
                f = follower[0]
                joints[name]["follower"] = f.name
                joints[name]["follower_multiplier"] = f.mimic["multiplier"]
                joints[name]["gravity_torque_follower_pinocchio"] = tau_g[f.name]
    mimic = {}
    for side in ("left", "right"):
        f1 = q[f"openarm_{side}_finger_joint1"]
        f2 = q[f"openarm_{side}_finger_joint2"]
        mimic[side] = f2 - f1
    return {"key": key, "controller_rate_hz": 1.0 / (period * m.opt.timestep), "controller_updates": updates,
            "physics_steps": steps, "simulated_s": steps * m.opt.timestep, "joints": joints, "mimic_error_rad": mimic}


def check_mimic_follow(mj, m, d, key, overrides, target_by_side, seconds=1.5, equality_active=True):
    """SW-09: drive finger_joint1 and measure finger_joint2. With equality_active False the constraint is switched off (control)."""
    period = controller_period_steps(m, overrides)
    mj.mj_resetDataKeyframe(m, d, m.key(key).id)
    for side in ("left", "right"):
        d.eq_active[m.equality(f"openarm_{side}_finger_joint2_mimic").id] = 1 if equality_active else 0
    steps = int(round(seconds / m.opt.timestep))
    worst = {"left": 0.0, "right": 0.0}
    for k in range(steps):
        if k % period == 0:
            for side, target in target_by_side.items():
                d.ctrl[m.actuator(f"openarm_{side}_finger_joint1_position").id] = target
        mj.mj_step(m, d)
        if k > steps // 2:
            for side in worst:
                f1 = d.qpos[m.jnt_qposadr[m.joint(f"openarm_{side}_finger_joint1").id]]
                f2 = d.qpos[m.jnt_qposadr[m.joint(f"openarm_{side}_finger_joint2").id]]
                worst[side] = max(worst[side], abs(float(f2 - f1)))
    final = {side: float(d.qpos[m.jnt_qposadr[m.joint(f"openarm_{side}_finger_joint1").id]]) for side in target_by_side}
    return {"key": key, "equality_active": equality_active, "finger1_final": final, "max_follow_error_rad": worst}


def overrides():
    from .mjcf import OVERRIDES
    return load_yaml(OVERRIDES)
