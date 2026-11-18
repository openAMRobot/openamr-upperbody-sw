# SPDX-License-Identifier: MIT
"""Grasp and lift of the target product in the MuJoCo grocery scene: the controller and the contact measurements.

MuJoCo experiment, not ROS or firmware integrated. The arm, gripper and lift are driven through the position actuators of the
generated model; this script is the only controller (no ros2_control, no MoveIt, no OpenArm driver, no firmware). It adds two things
the generated model does not have, both on the controller side and both declared:
  * gravity feed-forward on the seven arm joints (qfrc_applied = the model's own bias force on those joints), the MuJoCo analogue of
    the gravity feed-forward of pull request 73; without it the hand sags under the position servo gains;
  * the lift setpoint ramp at the decided 18 mm/s (LIFT), because the generated model does not enforce the lift speed (audits PR 89,
    check P07).
The model file is never changed. Sweep settings (product friction, grip command, gripper gain factor) are applied to the in-memory
model of one run only and are reported as experiment settings, never as proposed parameter values.
"""
import dataclasses
import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import store as st  # noqa: E402

G = 9.81


@dataclasses.dataclass
class Settings:
    friction: float = 1.0            # pair coefficient product/finger (the product geom has priority)
    grip_command: float = 0.0        # finger joint target when closed, rad (0 = fully closed, the Build 1 close command)
    kp_factor: float = 1.0           # gripper position gain, as a factor of the generated model's value
    open_command: float = 0.6        # finger joint target when open, rad
    gravity_ff: bool = True


@dataclasses.dataclass
class Phases:
    settle: float = 0.4
    approach: float = 1.6
    close: float = 0.6
    hold_before_lift: float = 0.6
    hold_after_lift: float = 1.0


class Pick:
    """One grasp-and-lift run. Construct, then call run() or step through phases with advance()."""

    def __init__(self, model, data, scene=None, settings=None, phases=None, side="left"):
        import mujoco
        self.mj = mujoco
        self.m, self.d = model, data
        self.scene = scene or st.load()
        self.s = settings or Settings()
        self.ph = phases or Phases()
        self.side = side
        m = model
        self.arm = [m.joint(f"openarm_{side}_joint{k}").id for k in range(1, 8)]
        self.qa = np.array([m.jnt_qposadr[j] for j in self.arm])
        self.va = np.array([m.jnt_dofadr[j] for j in self.arm])
        self.act_arm = [m.actuator(f"openarm_{side}_joint{k}_position").id for k in range(1, 8)]
        self.act_grip = m.actuator(f"openarm_{side}_finger_joint1_position").id
        self.act_lift = m.actuator("lift_joint_position").id
        self.finger_q = m.joint(f"openarm_{side}_finger_joint1").qposadr[0]
        self.lift_q = m.joint("lift_joint").qposadr[0]
        self.grasp_body = m.body(f"openarm_{side}_grasp_frame").id
        self.fingers = {m.geom(f"openarm_{side}_ee_link1__collision_0").id: "finger1",
                        m.geom(f"openarm_{side}_ee_link2__collision_0").id: "finger2"}
        hand_bodies = {m.body(f"openarm_{side}_{b}").id for b in ("link5", "link6", "ee_base_link", "ee_link1", "ee_link2")}
        self.hand_geoms = {g for g in range(m.ngeom) if m.geom_bodyid[g] in hand_bodies}
        self.product = m.body(self.scene["product"]["name"]).id
        self.product_geom = m.geom(f'{self.scene["product"]["name"]}_geom').id
        self.base_free = m.joint("base_free_joint").qposadr[0]
        self.product_free = m.joint(f'{self.scene["product"]["name"]}_free').qposadr[0]
        self.mass = float(st.value(self.scene["product"]["mass"]))
        pk = self.scene["pick"]
        self.lift_speed = st.value(pk["lift_speed"])
        self.lift_travel = st.value(pk["lift_travel"])
        self.base_kp = float(m.actuator_gainprm[self.act_grip, 0])
        self.base_bias = m.actuator_biasprm[self.act_grip, :3].copy()
        self._apply_settings()

    # ------------------------------------------------------------------ setup
    def _apply_settings(self):
        m, s = self.m, self.s
        m.geom_friction[self.product_geom, 0] = s.friction
        m.actuator_gainprm[self.act_grip, 0] = self.base_kp * s.kp_factor
        m.actuator_biasprm[self.act_grip, 1] = self.base_bias[1] * s.kp_factor

    def place_robot(self, pose):
        x, y, yaw = pose
        q = self.d.qpos
        q[self.base_free:self.base_free + 3] = [x, y, self.m.key_qpos[self.m.key("L1000_home").id][2]]
        q[self.base_free + 3:self.base_free + 7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]

    def ik(self, pos, rot, q0, iters=400):
        """Damped least squares on the seven arm joints for the grasp frame, base and lift as they are now. Kinematics only."""
        mj, m, d = self.mj, self.m, self.d
        saved = d.qpos.copy()
        q = saved.copy()
        q[self.qa] = q0
        lo, hi = m.jnt_range[self.arm, 0], m.jnt_range[self.arm, 1]
        err = (1.0, 1.0)
        for _ in range(iters):
            d.qpos[:] = q
            mj.mj_kinematics(m, d)
            mj.mj_comPos(m, d)
            ep = np.asarray(pos) - d.xpos[self.grasp_body]
            re = np.asarray(rot) @ d.xmat[self.grasp_body].reshape(3, 3).T
            quat = np.zeros(4)
            mj.mju_mat2Quat(quat, re.flatten())
            w = np.zeros(3)
            mj.mju_quat2Vel(w, quat, 1.0)
            jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
            mj.mj_jacBody(m, d, jp, jr, self.grasp_body)
            J = np.vstack([jp[:, self.va], jr[:, self.va]])
            e = np.concatenate([ep, w])
            err = (float(np.linalg.norm(ep)), float(np.linalg.norm(w)))
            if err[0] < 1e-6 and err[1] < 1e-5:
                break
            q[self.qa] = np.clip(q[self.qa] + J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e), lo, hi)
        d.qpos[:] = saved
        mj.mj_forward(m, d)
        return q[self.qa].copy(), err

    def _fk_grasp(self, q_arm):
        saved = self.d.qpos.copy()
        self.d.qpos[self.qa] = q_arm
        self.mj.mj_kinematics(self.m, self.d)
        pos = self.d.xpos[self.grasp_body].copy()
        self.d.qpos[:] = saved
        self.mj.mj_forward(self.m, self.d)
        return pos

    def targets(self, q_start=None):
        """Arm joint vectors along the straight approach, from the IK. The grasp frame x axis is the approach (world +x after the
        base yaw), y the closing direction, z up: a side grasp, square to the product face."""
        pk = self.scene["pick"]
        p = self.d.xpos[self.product].copy()
        yaw = self._base_yaw()
        c, s_ = math.cos(yaw), math.sin(yaw)
        rot = np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1.0]])
        grasp = p + np.array([0, 0, st.value(pk["grasp_height_above_product_centre"])])
        back = st.value(pk["approach_distance"])
        path = []
        q0 = np.array([-0.5, 0.0, 0.0, 1.6, 0.0, -0.5, 0.0]) if q_start is None else q_start
        for k in range(21):
            f = 1.0 - k / 20.0
            target = grasp - rot[:, 0] * back * f
            q0, err = self.ik(target, rot, q0)
            if err[0] > 1e-4 or err[1] > 1e-3:
                raise RuntimeError(f"IK did not converge at approach step {k}: position {err[0]:.2e} m, rotation {err[1]:.2e} rad")
            path.append(q0.copy())
        return np.array(path)

    def _base_yaw(self):
        w, _, _, z = self.d.qpos[self.base_free + 3:self.base_free + 7]
        return 2 * math.atan2(z, w)

    # ------------------------------------------------------------------ control
    def apply_ff(self):
        self.d.qfrc_applied[:] = 0
        if self.s.gravity_ff:
            self.d.qfrc_applied[self.va] = self.d.qfrc_bias[self.va]

    def set_arm(self, q):
        self.d.ctrl[self.act_arm] = q

    def contacts(self):
        """Forces on the product, in the world frame: per finger normal and tangential, the other hand geoms, the rest."""
        mj, m, d = self.mj, self.m, self.d
        out = {"finger1": [0.0, 0.0], "finger2": [0.0, 0.0], "hand_other": 0.0, "support": 0.0}
        f6 = np.zeros(6)
        for i in range(d.ncon):
            con = d.contact[i]
            pair = {con.geom1, con.geom2}
            if self.product_geom not in pair:
                continue
            other = (pair - {self.product_geom}).pop() if len(pair) == 2 else None
            mj.mj_contactForce(m, d, i, f6)
            normal, tangential = abs(f6[0]), float(np.hypot(f6[1], f6[2]))
            if other in self.fingers:
                key = self.fingers[other]
                out[key][0] += normal
                out[key][1] += tangential
            elif other in self.hand_geoms:
                out["hand_other"] += normal
            else:
                out["support"] += normal
        return out

    def product_in_grasp(self):
        """Product origin in the grasp frame (m)."""
        R = self.d.xmat[self.grasp_body].reshape(3, 3)
        return R.T @ (self.d.xpos[self.product] - self.d.xpos[self.grasp_body])

    def run(self, on_step=None, base_pose=None):
        """The whole sequence. Returns the measurements. on_step(t, phase) is called after every physics step."""
        mj, m, d = self.mj, self.m, self.d
        mj.mj_resetDataKeyframe(m, d, m.key("L1000_home").id)
        self.place_robot(base_pose or self.scene["robot"]["pick_station"]["pose"])
        mj.mj_forward(m, d)
        path = self.targets()
        d.qpos[self.qa] = path[0]
        d.qpos[self.finger_q] = self.s.open_command
        d.qpos[m.joint(f"openarm_{self.side}_finger_joint2").qposadr[0]] = self.s.open_command
        d.ctrl[:] = 0
        self.set_arm(path[0])
        d.ctrl[self.act_grip] = self.s.open_command
        d.ctrl[self.act_lift] = d.qpos[self.lift_q]
        mj.mj_forward(m, d)
        dt = m.opt.timestep
        rec = {"t": [], "phase": [], "product_z": [], "grasp_z": [], "rel": [], "finger_q": [], "lift_q": [], "forces": []}
        z0 = {}
        t = 0.0
        lift_start = float(d.qpos[self.lift_q])
        lift_time = self.lift_travel / self.lift_speed
        schedule = [("settle", self.ph.settle), ("approach", self.ph.approach), ("close", self.ph.close),
                    ("hold", self.ph.hold_before_lift), ("lift", lift_time), ("hold_lifted", self.ph.hold_after_lift)]
        for phase, duration in schedule:
            n = int(round(duration / dt))
            if phase == "approach":
                # The base settles on its wheel and castor contacts and pitches slightly under the extended arm: plan from the
                # settled base, not from the keyframe.
                path = self.targets(q_start=d.qpos[self.qa].copy())
                rec["approach_start_offset"] = float(np.linalg.norm(d.xpos[self.grasp_body] - self._fk_grasp(path[0])))
            for i in range(n):
                f = (i + 1) / n
                if phase == "approach":
                    k = f * (len(path) - 1)
                    j = min(int(k), len(path) - 2)
                    self.set_arm(path[j] + (k - j) * (path[j + 1] - path[j]))
                elif phase == "close":
                    d.ctrl[self.act_grip] = self.s.open_command + (self.s.grip_command - self.s.open_command) * min(1.0, 2 * f)
                elif phase == "lift":
                    d.ctrl[self.act_lift] = lift_start + self.lift_speed * (i + 1) * dt
                self.apply_ff()
                mj.mj_step(m, d)
                t += dt
                if on_step is not None:
                    on_step(t, phase)
                if i % 15 == 0 or i == n - 1:
                    rec["t"].append(t)
                    rec["phase"].append(phase)
                    rec["product_z"].append(float(d.xpos[self.product][2]))
                    rec["grasp_z"].append(float(d.xpos[self.grasp_body][2]))
                    rec["rel"].append(self.product_in_grasp().tolist())
                    rec["finger_q"].append(float(d.qpos[self.finger_q]))
                    rec["lift_q"].append(float(d.qpos[self.lift_q]))
                    rec["forces"].append(self.contacts())
            z0[phase] = {"product_z": float(d.xpos[self.product][2]), "grasp_z": float(d.xpos[self.grasp_body][2]),
                         "rel": self.product_in_grasp().tolist(), "lift_q": float(d.qpos[self.lift_q]),
                         "finger_q": float(d.qpos[self.finger_q])}
        return self.summarise(rec, z0)

    def summarise(self, rec, end):
        ph = np.array(rec["phase"])
        lifted = (ph == "lift") | (ph == "hold_lifted")
        held = ph == "hold_lifted"
        rel = np.array(rec["rel"])
        n1 = np.array([f["finger1"][0] for f in rec["forces"]])
        n2 = np.array([f["finger2"][0] for f in rec["forces"]])
        t1 = np.array([f["finger1"][1] for f in rec["forces"]])
        t2 = np.array([f["finger2"][1] for f in rec["forces"]])
        hand = np.array([f["hand_other"] for f in rec["forces"]])
        rel_hold = np.array(end["hold"]["rel"])
        slip = rel[lifted] - rel_hold
        product_rise = end["hold_lifted"]["product_z"] - end["hold"]["product_z"]
        grasp_rise = end["hold_lifted"]["grasp_z"] - end["hold"]["grasp_z"]
        need = self.mass * G / (2 * self.s.friction)
        mean_normal = float(np.mean((n1 + n2)[held]) / 2) if held.any() else float("nan")
        return {
            "settings": dataclasses.asdict(self.s),
            "gripper_kp": self.base_kp * self.s.kp_factor,
            "finger_q_at_contact_hold": end["hold"]["finger_q"],
            "grip_torque_hold": self.base_kp * self.s.kp_factor * (end["hold"]["finger_q"] - self.s.grip_command),
            "normal_force_per_finger_held_mean": mean_normal,
            "normal_force_finger1_held_mean": float(np.mean(n1[held])) if held.any() else float("nan"),
            "normal_force_finger2_held_mean": float(np.mean(n2[held])) if held.any() else float("nan"),
            "tangential_force_held_mean": float(np.mean((t1 + t2)[held])) if held.any() else float("nan"),
            "hand_other_contact_max": float(hand[lifted].max()) if lifted.any() else 0.0,
            "static_need_per_finger": need,
            "margin": mean_normal / need if need > 0 else float("nan"),
            "lift_travel": end["hold_lifted"]["lift_q"] - end["hold"]["lift_q"],
            "grasp_rise": grasp_rise,
            "product_rise": product_rise,
            "slip_z_max": float(-slip[:, 2].min()) if len(slip) else float("nan"),
            "slip_total_end": float(np.linalg.norm(np.array(end["hold_lifted"]["rel"]) - rel_hold)),
            "slip_rate_hold": float((end["lift"]["rel"][2] - end["hold_lifted"]["rel"][2]) / self.ph.hold_after_lift),
            "approach_start_offset": rec.get("approach_start_offset"),
            "held": bool(product_rise > 0.8 * grasp_rise and grasp_rise > 0.02),
            "series": rec,
        }
