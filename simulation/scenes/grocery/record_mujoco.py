#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Recording of the MuJoCo grocery scene: a camera path through the aisles and the robot's pick of the target product.

  MUJOCO_GL=egl python3 simulation/scenes/grocery/record_mujoco.py --scene build/grocery/mujoco/grocery_scene.xml \
      --out build/grocery/video

What is simulated and what is not, also written into every frame:
  * base drive and turn: KINEMATIC. The base pose is set frame by frame (qpos), wheels turned by distance over radius; no wheel
    physics, no Nav2, no controller. Speed at most 0.5 m/s (decisions.yaml SPEED-CEILING).
  * arm to the pre-grasp pose during the drive: KINEMATIC (joint interpolation).
  * the pick: CONTROLLED in MuJoCo physics: pick.py drives the position actuators of the generated model (gravity feed-forward on the
    arm joints, lift ramp 18 mm/s), friction 1.0 and close command 0 (the Build 1 values). Real time, one frame every 1/30 s.
MuJoCo only: not ROS, not ros2_control, not firmware integrated. Writes grocery_pick.mp4 (H.264, needs ffmpeg on PATH), stills
still_*.png (both through ffmpeg) and timeline.json.
"""
import argparse
import json
import math
import pathlib
import shutil
import subprocess
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import pick  # noqa: E402
import store as st  # noqa: E402

W, H, FPS = 1280, 720, 30
FOOTER = "MuJoCo experiment - not ROS or firmware integrated"


def smooth(s):
    s = min(max(s, 0.0), 1.0)
    return s * s * (3 - 2 * s)


def lerp(a, b, s):
    return np.asarray(a, float) + (np.asarray(b, float) - np.asarray(a, float)) * s


class Recorder:
    def __init__(self, model, data, out):
        import mujoco
        self.mj, self.m, self.d = mujoco, model, data
        model.vis.global_.offwidth = max(model.vis.global_.offwidth, W)
        model.vis.global_.offheight = max(model.vis.global_.offheight, H)
        self.r = mujoco.Renderer(model, H, W)
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.opt = mujoco.MjvOption()
        self.out = pathlib.Path(out)
        self.frames = 0
        self.timeline = []
        self.ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
                                    "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-preset", "medium", "-crf", "20",
                                    "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(self.out / "grocery_pick.mp4")],
                                   stdin=subprocess.PIPE)
        self.stills = {}

    def set_cam(self, lookat, distance, azimuth, elevation):
        self.cam.lookat[:] = lookat
        self.cam.distance, self.cam.azimuth, self.cam.elevation = distance, azimuth, elevation

    def frame(self, caption, still=None):
        mj = self.mj
        self.r.update_scene(self.d, self.cam, self.opt)
        ctx = self.r._mjr_context  # the Renderer's own context; overlays need the low-level calls
        rect = mj.MjrRect(0, 0, W, H)
        mj.mjr_setBuffer(mj.mjtFramebuffer.mjFB_OFFSCREEN, ctx)
        mj.mjr_render(rect, self.r.scene, ctx)
        mj.mjr_overlay(mj.mjtFont.mjFONT_NORMAL, mj.mjtGridPos.mjGRID_TOPLEFT, rect, caption, "", ctx)
        mj.mjr_overlay(mj.mjtFont.mjFONT_NORMAL, mj.mjtGridPos.mjGRID_BOTTOMRIGHT, rect, FOOTER, "", ctx)
        img = np.empty((H, W, 3), np.uint8)
        mj.mjr_readPixels(img, None, rect, ctx)
        img = np.ascontiguousarray(np.flipud(img))
        self.ff.stdin.write(img.tobytes())
        if still:
            path = self.out / f"still_{still}.png"
            subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-i", "-",
                            "-frames:v", "1", str(path)], input=img.tobytes(), check=True)
            self.stills[still] = path.name
        self.frames += 1

    def close(self):
        self.ff.stdin.close()
        rc = self.ff.wait()
        if rc != 0:
            raise RuntimeError(f"ffmpeg exited {rc}")


def base_qpos(p, x, y, yaw, travelled, turned):
    """Kinematic base pose and wheel angles (distance over radius, differential for the turn)."""
    d, m = p.d, p.m
    p.place_robot((x, y, yaw))
    radius = m.geom_size[m.geom("left_wheel_link__collision_0").id][0]
    track = abs(m.body_pos[m.body("left_wheel_link").id][1] - m.body_pos[m.body("right_wheel_link").id][1])
    d.qpos[m.joint("left_wheel_joint").qposadr[0]] = (travelled - turned * track / 2) / radius
    d.qpos[m.joint("right_wheel_joint").qposadr[0]] = (travelled + turned * track / 2) / radius


def record(scene_path, out_dir):
    import mujoco
    out = pathlib.Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    if shutil.which("ffmpeg") is None:
        raise RuntimeError("BLOCKED: ffmpeg is not on PATH")
    m = mujoco.MjModel.from_xml_path(str(scene_path))
    d = mujoco.MjData(m)
    data = st.load()
    p = pick.Pick(m, d, scene=data)
    rec = Recorder(m, d, out)
    home = m.key_qpos[m.key("L1000_home").id].copy()
    # The pre-grasp arm pose: the first approach point, from the robot standing at the pick station.
    mujoco.mj_resetDataKeyframe(m, d, m.key("L1000_home").id)
    p.place_robot(data["robot"]["pick_station"]["pose"])
    mujoco.mj_forward(m, d)
    q_pre = p.targets()[0]
    finger2 = m.joint(f"openarm_{p.side}_finger_joint2").qposadr[0]

    px, py, pyaw = data["robot"]["parking_spot"]["pose"]
    sx, sy, syaw = data["robot"]["pick_station"]["pose"]
    drive_len = math.hypot(sx - px, sy - py)
    speed = 0.45                                    # below the 0.5 m/s Build 1 ceiling
    t_drive = drive_len / speed
    t_turn = 2.5
    product = d.xpos[p.product].copy()
    shots = []

    def kin_state(x, y, yaw, travelled, turned, arm_s):
        d.qpos[:] = home
        base_qpos(p, x, y, yaw, travelled, turned)
        q = lerp(home[p.qa], q_pre, smooth(arm_s))
        d.qpos[p.qa] = q
        d.qpos[p.finger_q] = d.qpos[finger2] = 0.6 * smooth(arm_s)
        mujoco.mj_forward(m, d)

    # A. Establishing shot, robot parked.
    n = int(4.0 * FPS)
    for i in range(n):
        s = smooth(i / (n - 1))
        kin_state(px, py, pyaw, 0.0, 0.0, 0.0)
        rec.set_cam(lerp((1.6, 0.0, 0.5), (0.2, -2.4, 0.6), s), 8.5 - 3.0 * s, 35 + 20 * s, -38 + 10 * s)
        rec.frame("Grocery dark store, robot at its parking spot (kinematic)", still="1_store" if i == n - 1 else None)
    shots.append({"segment": "establishing", "frames": n, "mode": "kinematic"})
    # B. Drive along aisle 1 (kinematic), then turn to face the shelf while the arm moves to the pre-grasp pose (kinematic).
    n = int(t_drive * FPS)
    for i in range(n):
        s = i / (n - 1)
        f = smooth(s)
        x, y = px + (sx - px) * f, py + (sy - py) * f
        kin_state(x, y, pyaw, drive_len * f, 0.0, 0.0)
        rec.set_cam((x + 0.2, y + 0.6, 0.75), 3.4 - 0.6 * s, 70 + 25 * s, -22 + 4 * s)
        rec.frame("Drive along aisle 1: KINEMATIC base motion (pose set per frame, no wheel physics)",
                  still="2_aisle" if i == n // 2 else None)
    shots.append({"segment": "drive", "frames": n, "mode": "kinematic", "distance_m": drive_len, "speed_m_s": speed})
    n = int(t_turn * FPS)
    for i in range(n):
        s = i / (n - 1)
        yaw = pyaw + (syaw - pyaw) * smooth(s)
        kin_state(sx, sy, yaw, drive_len, (yaw - pyaw), s)
        rec.set_cam(lerp((sx + 0.2, sy + 0.6, 0.75), product + np.array([-0.15, 0, 0.0]), smooth(s)),
                    2.8 - 1.2 * smooth(s), 95 - 151 * smooth(s), -18 - 4 * smooth(s))
        rec.frame("Turn to the shelf, arm to pre-grasp: KINEMATIC")
    shots.append({"segment": "turn", "frames": n, "mode": "kinematic"})
    # C. The pick, controlled in MuJoCo physics, real time.
    state = {"next": 0.0, "n": 0}
    captions = {"settle": "Pick: MuJoCo physics, position actuators of the generated model",
                "approach": "Approach square to the product face (IK, position actuators)",
                "close": "Close the gripper (position servo, close command 0)",
                "hold": "Grip on the product (friction 1.0, 0.15 kg, Build 1 values)",
                "lift": "Lift actuator 18 mm/s (ramp in the controller; the model does not enforce it)",
                "hold_lifted": "Product held: MuJoCo contact result, not a hardware result"}
    still_at = {"approach": "3_approach", "lift": "4_lift", "hold_lifted": "5_held"}
    taken = set()
    total = sum((p.ph.settle, p.ph.approach, p.ph.close, p.ph.hold_before_lift, p.lift_travel / p.lift_speed, p.ph.hold_after_lift))

    def on_step(t, phase):
        if t + 1e-9 < state["next"]:
            return
        state["next"] += 1.0 / FPS
        s = smooth(t / total)
        prod = d.xpos[p.product]
        rec.set_cam(lerp(product + np.array([-0.15, 0, 0.0]), prod + np.array([-0.05, 0.02, 0.03]), s), 1.6 - 0.75 * s,
                    -56 + 28 * s, -22 + 10 * s)
        still = None
        if phase in still_at and phase not in taken and (phase != "lift" or t > total - p.ph.hold_after_lift - 0.5):
            if phase != "hold_lifted" or t > total - 0.2:
                still = still_at[phase]
                taken.add(phase)
        rec.frame(captions[phase], still=still)
        state["n"] += 1

    result = p.run(on_step=on_step)
    result.pop("series")
    shots.append({"segment": "pick", "frames": state["n"], "mode": "controlled (MuJoCo physics, real time)", "result": result})
    # D. Pull back over the store, robot holding the product (state frozen: no further stepping).
    n = int(4.0 * FPS)
    start = (d.xpos[p.product] + np.array([-0.05, 0.02, 0.03]), 0.85, -28.0, -12.0)
    for i in range(n):
        s = smooth(i / (n - 1))
        rec.set_cam(lerp(start[0], (1.3, 0.6, 0.5), s), start[1] + 6.2 * s, start[2] + 85 * s, start[3] - 26 * s)
        rec.frame("Product held; packing tote at the end of aisle 1 (scene dressing)", still="6_overview" if i == n - 1 else None)
    shots.append({"segment": "outro", "frames": n, "mode": "frozen state, camera only"})
    rec.close()
    timeline = {"fps": FPS, "size": [W, H], "frames": rec.frames, "duration_s": rec.frames / FPS, "segments": shots,
                "stills": rec.stills, "label": FOOTER}
    (out / "timeline.json").write_text(json.dumps(timeline, indent=1, default=float) + "\n")
    return timeline


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scene", required=True)
    parser.add_argument("--out", default="build/grocery/video")
    opts = parser.parse_args()
    t = record(opts.scene, opts.out)
    print(json.dumps({k: t[k] for k in ("frames", "duration_s", "stills")}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
