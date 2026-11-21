# SPDX-License-Identifier: MIT
"""Arm gravity feed-forward in the MuJoCo control path, the counterpart of the Build 1 real-driver path.

The real stack (upperbody-sw pull request 73, carried by integration/baseline1: ros2/src/openamrobot_build1_scene,
gravity.py and scripts/gravity_feedforward.py, launch argument gravity_feedforward default true) sends every arm motor one
MIT frame {kp, kd, q, dq, tau}. The trajectory controller owns q (position interface), the gravity controller owns tau
(effort interface, fed by gravity_feedforward), kp and kd are the driver's own gains. The node publishes per arm, at
100 Hz, the static holding torque at the measured joint positions when all seven joints of that arm have a state younger
than max_state_age_s (0.2 s), and zeros otherwise (docs/architecture/d4_mit_frame_field_ownership.md on that branch).

Here the same split, in MuJoCo:
  q, dq   the position actuators of the MJCF (kp, kd of the driver, engine_overrides.yaml arm_position_gains), unchanged
  tau     this module: the gravity term of the MuJoCo model at the measured arm positions with the base level and at rest
          (the real node's model is the URDF in the root frame base_footprint on a level floor), zero velocity, no payload,
          added as a joint torque (qfrc_applied) and held between updates, as the effort controller holds its last command

The masses are the MJCF's, which equal the expansion's (parity A4). Gravity is the model's (engine_overrides.yaml common
gravity, 9.81 m/s^2); the real node uses 9.80665 m/s^2, 0.034 % less. A held object (the real node's payload) is not
modelled: no benchmark scenario holds one. Not a safety function, not a dynamics model, not safety evidence.
"""
MODES = ("build1", "off")
ARM_JOINTS = {side: [f"openarm_{side}_joint{i}" for i in range(1, 8)] for side in ("left", "right")}


def settings(overrides):
    """The declared feed-forward settings of engine_overrides.yaml (actuators.arm_gravity_feedforward)."""
    ff = overrides["actuators"]["arm_gravity_feedforward"]
    mode = ff["mode"]["value"]
    if mode not in MODES:
        raise ValueError(f"arm_gravity_feedforward mode {mode!r} is not one of {MODES}")
    return {"mode": mode, "rate_hz": float(ff["rate_hz"]["value"]), "max_state_age_s": float(ff["max_state_age_s"]["value"])}


class GravityFeedForward:
    """What gravity_feedforward sends each cycle, per arm, computed on the MuJoCo model. No ROS import.

    on_state(d, now_s) records the measured arm positions (the joint state broadcaster's message); command(now_s) returns
    {side: [tau of joint1..7]}: the model torque for an arm whose seven joints are all younger than max_state_age_s, zeros
    for any other arm, never received included. apply(d, commands) writes them to the joints as qfrc_applied."""

    def __init__(self, m, max_state_age_s, arms=None):
        import mujoco as mj
        if not max_state_age_s > 0.0:
            raise ValueError(f"max_state_age_s must be positive, got {max_state_age_s}")
        self.mj, self.m, self.max_age_s = mj, m, float(max_state_age_s)
        self.arms = ARM_JOINTS if arms is None else arms
        self.scratch = mj.MjData(m)
        self.qadr = {jn: int(m.jnt_qposadr[m.joint(jn).id]) for joints in self.arms.values() for jn in joints}
        self.dof = {jn: int(m.jnt_dofadr[m.joint(jn).id]) for joints in self.arms.values() for jn in joints}
        self.free = [j for j in range(m.njnt) if m.jnt_type[j] == mj.mjtJoint.mjJNT_FREE]
        self.qpos, self.received = None, {}

    def on_state(self, d, now_s):
        self.qpos = d.qpos.copy()
        for jn in self.qadr:
            self.received[jn] = float(now_s)

    def fresh(self, side, now_s):
        return all(jn in self.received and now_s - self.received[jn] <= self.max_age_s for jn in self.arms[side])

    def torques(self, qpos):
        """Holding torque of every arm joint at qpos: the model's bias force with zero velocity, base level and at rest."""
        import numpy as np  # here, not at module level: the comparison job imports benchmark.py with PyYAML only
        s = self.scratch
        s.qpos[:] = qpos
        for j in self.free:  # base level: identity orientation, as the real node's root frame on a level floor
            a = self.m.jnt_qposadr[j]
            s.qpos[a + 3:a + 7] = (1.0, 0.0, 0.0, 0.0)
        s.qvel[:] = 0.0
        self.mj.mj_kinematics(self.m, s)
        self.mj.mj_comPos(self.m, s)
        self.mj.mj_comVel(self.m, s)
        bias = np.zeros(self.m.nv)
        self.mj.mj_rne(self.m, s, 0, bias)  # flg_acc 0, qvel 0: the gravity term only
        return {jn: float(bias[self.dof[jn]]) for jn in self.dof}

    def command(self, now_s):
        """-> ({side: torques}, [stale sides]). Every arm gets exactly one vector each cycle."""
        fresh = [side for side in self.arms if self.fresh(side, now_s)]
        out = {side: [0.0] * len(joints) for side, joints in self.arms.items()}
        if fresh:
            tau = self.torques(self.qpos)
            for side in fresh:
                out[side] = [tau[jn] for jn in self.arms[side]]
        return out, [side for side in self.arms if side not in fresh]

    def apply(self, d, commands):
        for side, values in commands.items():
            for jn, value in zip(self.arms[side], values):
                d.qfrc_applied[self.dof[jn]] = value
