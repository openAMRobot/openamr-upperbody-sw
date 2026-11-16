#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""The short task of the scene video, through the scene's existing commands and the rig of openamrobot_build1_scene:
/cmd_vel (DiffDrive of the base in profile fast), lift_controller, the two arm trajectory controllers and the two
gripper controllers. No new controller. Writes OUT_DIR/task.json with the Gazebo time of each step and what each
command returned. Visual capture, not a test result: nothing here is checked against a limit.

    video_task.py OUT_DIR
"""
import json
import os
import sys
import time

from openamrobot_build1_scene import rig as rig_module
from openamrobot_build1_scene.scenarios import CmdVel, PARK

HANDS_UP = [0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0]     # SRDF group_state hands_up of openamrobot_upperbody_moveit
OPEN, CLOSED = 0.7854, 0.0


def main(out_dir):
    os.makedirs(out_dir, exist_ok=True)
    rig = rig_module.Rig("scene_video", node_name="scene_video_task")
    record = {"note": "visual capture, not a test result", "steps": [], "markers": {}}

    def step(name, **data):
        entry = {"name": name, "sim_s": round(rig.sim_time(), 3), "rtf_last": rig.rtf(), **data}
        record["steps"].append(entry)
        print(f"VIDEO_TASK {json.dumps(entry)}", flush=True)

    def mark(name):
        record["markers"][name] = round(rig.sim_time(), 3)

    def arms(positions, seconds, label):
        handles = {side: rig.follow_joint_trajectory(side, positions, seconds) for side in ("left", "right")}
        result = {side: (rig.succeeded(rig.wait_result(h, seconds + 20.0, f"{side} {label}")[0]) if h else None)
                  for side, h in handles.items()}
        step(label, succeeded=result)

    try:
        rig.wait_ready(arms=True, lift=True)
        step("ready", run_info=rig.get("/build1/run_info"))
        arms(PARK, 3.0, "arms_activation_to_park")
        rig.sleep_sim(1.0)
        mark("start")
        step("start", truth=rig.truth())

        drive = CmdVel(rig)
        drive.command = (0.2, 0.0)
        rig.sleep_sim(8.0)
        drive.command = (0.0, 0.0)
        rig.sleep_sim(1.0)
        step("drive_forward_0.2_m_s_for_8_s", truth=rig.truth())
        drive.command = (0.0, -0.5)
        rig.sleep_sim(3.14)
        drive.stop()
        rig.sleep_sim(1.0)
        step("turn_right_0.5_rad_s_for_3.14_s", truth=rig.truth())

        for attempt in (1, 2):      # README Known limits: in DART the lift did not move on some commands
            lifted = rig.lift_move(2)
            step(f"lift_to_stop_2_L1350_attempt_{attempt}", succeeded=lifted, lift_m=rig.lift_truth())
            if rig.lift_truth() > 0.3:
                break

        arms(HANDS_UP, 3.0, "arms_park_to_hands_up")
        rig.sleep_sim(1.0)
        mark("reach")
        for side in ("left", "right"):
            closed = rig.gripper(side, CLOSED, sim_s=6.0)
            step(f"{side}_gripper_close", result=closed)
        rig.sleep_sim(0.5)
        for side in ("left", "right"):
            opened = rig.gripper(side, OPEN, sim_s=6.0)
            step(f"{side}_gripper_open", result=opened)
        arms(PARK, 3.0, "arms_hands_up_to_park")
        rig.sleep_sim(1.5)
        mark("end")
        step("end", truth=rig.truth(), lift_m=rig.lift_truth(), rtf_stats=rig.get("/build1/rtf_stats"))
        rc = 0
    except Exception as error:     # a visual capture records what happened and ends
        step("error", text=repr(error))
        mark("end")
        rc = 1
    finally:
        with open(os.path.join(out_dir, "task.json"), "w", encoding="utf-8") as f:
            json.dump(record, f, indent=2, default=str)
        time.sleep(0.5)
        rig.close()
    return rc


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
