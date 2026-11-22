# SPDX-License-Identifier: MIT
"""Title cards (1920x1080 PNG) for Videos B and C. Text only; every number is copied from the run's report JSON."""
import os, sys, json
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
BG, FG, FG2, MUTED, GOOD, BAD, ACC = "#1a1a19", "#f0efec", "#c3c2b7", "#8d8c86", "#0ca30c", "#e66767", "#3987e5"

def card(path, title, lines, foot=None, size=40):
    fig = plt.figure(figsize=(19.2, 10.8), dpi=100, facecolor=BG)
    fig.text(0.06, 0.86, title, fontsize=size, fontweight="bold", color=FG, family="DejaVu Sans", va="top")
    y = 0.74
    for ln in lines:
        if ln is None: y -= 0.025; continue
        txt, kw = (ln, {}) if isinstance(ln, str) else ln
        st = dict(fontsize=19, color=FG2, family="DejaVu Sans", va="top"); st.update(kw)
        fig.text(0.06, y, txt, **st); y -= 0.024 * st["fontsize"] / 10
    if foot: fig.text(0.06, 0.05, foot, fontsize=14, color=MUTED, family="DejaVu Sans", va="bottom", linespacing=1.5)
    fig.savefig(path, dpi=100, facecolor=BG); plt.close(fig)

RUN_ID = os.environ.get("VIDEO_RUN", "44309791416"); LOG = os.environ.get("VIDEO_LOG", "in_b1/ws/build1_logs")
REF = os.environ.get("VIDEO_REF", "integration/baseline1 d63e9ed  |  platform-sw 931e631")
CHAIN = os.environ.get("VIDEO_CHAIN", "")       # one line describing the base command chain of the recorded run
SPEED = float(os.environ.get("VIDEO_SPEED", "1"))
RUN_C = "https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/" + RUN_ID
RUN_B = "https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/44299879412"
which = sys.argv[1]
if which == "c":
    card("card_c_intro.png", "Engineering digital twin explained", [
        ("One recorded run of the Baseline 1 mission: run 44309791416, 8 of 8 phases PASS.", {"color": FG, "fontsize": 22}), None,
        "Left: the fixed observer camera rendered by Gazebo, one frame per second of Gazebo time.",
        "Right: panels drawn from the recorded ROS 2 bag, the candump logs of the five virtual CAN buses",
        "and the mission's phase table. Nothing is re-simulated; no signal is drawn that is not in the recording.", None,
        "Gazebo Harmonic (DART physics) with the ROS 2 Jazzy stack: Nav2, AMCL, MoveIt 2, ros2_control.",
        "Host emulators on virtual CAN: ZLAC8015D drive, CTL-017 lift controller, Daly BMS, Damiao arm motors. STM32: host stand-in.", None,
        ("Profile: simulation-oracle (pose_source: ground_truth, object_pose_source: ground_truth).", {"color": FG}),
        "Planning scene and object poses come from Gazebo ground truth, not from perception.",
        ("Simulation evidence only: not hardware, perception or safety validation.", {"color": BAD}), None,
        ("The observer camera is 480 x 270 at 1 frame per Gazebo second; it is shown enlarged, which adds no detail.", {"fontsize": 17}),
    ], foot=f"Source: {RUN_C}  |  integration/baseline1 d63e9ed  |  platform-sw 931e631")
    d = json.load(open("in_b1/ws/build1_logs/baseline1_mission.json"))
    q = {x["name"]: x for p in d["phases"] for x in p["quantities"]}
    v = lambda n, s=1: q[n]["value"] * s
    card("card_c_outro.png", "Result of run 44309791416: 8 of 8 phases PASS", [
        ("spawn, nav_to_station, stop_authorize, lift_to_target, arm_approach, grasp, lift_object, retreat", {"color": GOOD}), None,
        f"Station reached within {v('station_reach_position_error', 1000):.1f} mm and {v('station_reach_yaw_error', 1000):.0f} mrad of the goal (limits 40 mm, 80 mrad), Gazebo ground truth.",
        f"Wheel odometry minus ground truth after the drive: {v('nav_odometry_minus_ground_truth', 1000):.0f} mm (limit 100 mm).",
        f"Lift stop 1 at {v('lift_stop_1_gazebo_position', 1000):.1f} mm in Gazebo, {v('lift_stop_1_joint_state_position', 1000):.0f} mm in the joint state read over CAN (window 169 to 181 mm).",
        f"Item lifted {v('item_height_gain_after_lift', 1000):.1f} mm after the grasp (window 40 to 120 mm), held without drift ({v('item_height_drift_while_held', 1000):+.2f} mm).", None,
        ("Limitations of this video", {"color": FG, "fontsize": 21}),
        "Observer camera: 1 Hz of Gazebo time, each frame held 1 s; frame k is matched to Gazebo time 8 + k s",
        "(fitted to the ground-truth base motion, within about 0.3 s). CAN host time is mapped to Gazebo time through the",
        "mission trace (0.5 s samples). The odom TF was not recorded: odometry shown is the STM32 stand-in's /sil/drive/status.",
        "Contact forces and gripper action status were not recorded in this run. Real-time factor about 0.2.",
    ], foot=f"Run: {RUN_C}   |   simulation only, not hardware validation", size=36)
if which == "short":
    d = json.load(open(f"{LOG}/baseline1_mission.json"))
    ph = [(p["phase"], p["ran"] and not p["failures"]) for p in d["phases"]]; n_ok = sum(ok for _, ok in ph)
    q = {x["name"]: x for p in d["phases"] for x in p["quantities"]}
    def line(fmt, name, scale=1000):
        return fmt.format(q[name]["value"] * scale) if name in q else None
    card("card_s_intro.png", "OpenAMRobot 2.0 digital twin: one recorded mission", [
        (f"Run {RUN_ID}: {n_ok} of {len(ph)} mission phases PASS. Played at {SPEED:g}x Gazebo time.", {"color": FG, "fontsize": 24}), None,
        ("Left: Gazebo observer camera. Right: panels drawn from the recorded ROS 2 bag, the candump logs", {"fontsize": 20}),
        ("of the virtual CAN buses and the mission's own phase table. Nothing is re-simulated.", {"fontsize": 20}), None,
        *([(CHAIN, {"fontsize": 19, "color": FG})] if CHAIN else []),
        ("Simulation-oracle profile: object and robot poses come from Gazebo ground truth, not perception.", {"fontsize": 19}),
        ("Simulation evidence only: not hardware, perception or safety validation.", {"color": BAD, "fontsize": 20}),
    ], foot=f"Source: {RUN_C}\n{REF}", size=38)
    res = [x for x in (
        line("Station reached within {:.1f} mm of the goal (limit 40 mm), Gazebo ground truth.", "station_reach_position_error"),
        line("Wheel odometry minus ground truth after the drive: {:.0f} mm (limit 100 mm).", "nav_odometry_minus_ground_truth"),
        line("Item lifted {:.1f} mm after the grasp (window 40 to 120 mm).", "item_height_gain_after_lift")) if x]
    card("card_s_outro.png", f"Result of run {RUN_ID}: {n_ok} of {len(ph)} phases PASS", [
        (", ".join(n for n, _ in ph), {"color": GOOD if n_ok == len(ph) else BAD, "fontsize": 18}), None,
        *[(r, {"fontsize": 20}) for r in res], None,
        ("Not shown and not claimed: hardware, perception, safety functions, firmware on the target MCU.", {"fontsize": 19}),
        ("Device emulators run on the host over virtual CAN. Real-time factor is below 1.", {"fontsize": 19}),
    ], foot=f"Source: {RUN_C}\n{REF}  |  rendering: tools/video on diag/rc1-video-2026-11-22", size=36)
if which == "b":
    d = json.load(open("in_gd/ws/build1_logs/grocery_demo_report.json")); g = d["grocery"]
    n_ok = sum(p["verdict"] == "PASS" for p in d["phases"]); n = len(d["phases"]); assert (n_ok, n) == (10, 12)
    card("card_b_intro.png", "Grocery warehouse demonstration", [
        ("A complete recorded simulation run of the OpenAMRobot digital twin: run 44299879412.", {"color": FG, "fontsize": 23}), None,
        ("Task: drive to a store shelf, raise the lift, pick one product with the left arm, carry it", {"fontsize": 21}),
        ("to the packing station, place it into the order tote and return to the parking bay.", {"fontsize": 21}), None,
        (f"Result: {n_ok} of {n} phase checks passed. 2 failed: shelf parking 49.7 mm (limit 40 mm)", {"fontsize": 21, "color": BAD}),
        ("and the odometry guard on the drive to packing (0.907 m, guard 0.56 m).", {"fontsize": 21, "color": BAD}), None,
        ("Gazebo simulation, simulation-oracle profile: planning scene and object poses from ground truth.", {"fontsize": 19}),
        ("Not hardware, perception or safety validation.", {"fontsize": 19}),
    ], foot=f"Source: {RUN_B}  |  demo/grocery-dark-store 6170e6a")
    card("card_b_outro.png", f"Outcome: {n_ok} of {n} checks passed", [
        ("Demonstrated in simulation: Nav2 navigation with AMCL on LiDAR, a lift and two arms driven", {"fontsize": 20}),
        ("through emulated devices on virtual CAN, MoveIt 2 pick and place with contact physics.", {"fontsize": 20}), None,
        ("Failed: shelf parking 49.7 mm (limit 40 mm); odometry guard 0.907 m (guard 0.56 m).", {"fontsize": 20, "color": BAD}), None,
        ("Not shown: hardware, perception (poses are ground truth), safety functions, firmware (STM32 is a host stand-in).", {"fontsize": 17}),
    ], foot=f"Source: {RUN_B}  |  rendering: openamr-upperbody-sw diag/sim-videos tools/video", size=38)
