# SPDX-License-Identifier: MIT
"""Technical slides of Video C, each drawn from the recorded data of Baseline 1 run 44309791416."""
import sys, json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
sys.path.insert(0, __file__.rsplit("/", 1)[0]); import candecode as cd
BG, FG, FG2, MUTED, GRID = "#1a1a19", "#f0efec", "#c3c2b7", "#8d8c86", "#2e2e2c"
C = ["#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#0ca30c", "#9085e9", "#e66767"]; GOOD, BAD = "#0ca30c", "#e66767"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 13, "text.color": FG, "axes.edgecolor": "#5a5955", "xtick.color": FG2,
                     "ytick.color": FG2, "axes.facecolor": BG, "figure.facecolor": BG, "axes.labelcolor": FG2, "axes.grid": True,
                     "grid.color": GRID, "lines.linewidth": 2, "legend.frameon": False, "legend.fontsize": 11})
LOG = "in_b1/ws/build1_logs"; M = json.load(open(f"{LOG}/baseline1_mission.json"))
z = np.load("b1.npz", allow_pickle=True); can = np.load("b1_can.npz"); x = np.load("b1x.npz")
SRC = "Source: openamr-upperbody-sw run 44309791416 (integration/baseline1 d63e9ed), openamr-platform-sw 931e631, openarm_can bfd4e52, openarm_ros2 4e837e1"
def page(title, sub):
    f = plt.figure(figsize=(19.2, 10.8), dpi=100)
    f.text(0.03, 0.95, title, fontsize=26, fontweight="bold", va="center"); f.text(0.03, 0.905, sub, fontsize=14, color=FG2, va="center")
    f.text(0.97, 0.015, SRC, fontsize=10, color=MUTED, ha="right"); return f
def save(f, n): f.savefig(n, dpi=100); plt.close(f)

# T1 components and data flow
f = page("What runs in this digital twin", "Baseline 1 mission, profile firmware_sil, Gazebo simulation, simulation-oracle (pose_source: ground_truth, object_pose_source: ground_truth)")
rows = [("Physics", "Gazebo Harmonic, DART (gz-physics-dartsim-plugin), 1 ms step: robot, shelf, tray, product, wheels and contacts"),
        ("Navigation", "Nav2 (navigate_to_station.xml) + AMCL on /scan_filtered; planning scene from Gazebo ground truth (oracle)"),
        ("Base commands", "Nav2 -> velocity smoother -> collision_monitor, the single /cmd_vel writer. No command arbiter in these runs"),
        ("STM32 base controller", "host stand-in (stm32: host): plays the CAN role of the STM32; it is not the firmware and not Renode"),
        ("vcan_drive", "ZLAC8015D drive emulator, CANopen node 1: SDO target velocity 0x60FF, fed by Gazebo wheel states"),
        ("vcan_aux", "CTL-017 lift controller, host-side emulator, CANopen node 0x10: RPDO1 command, TPDO1/TPDO2 status, permit"),
        ("vcan_bms", "Daly BMS emulator: 29-bit request/reply frames (0x90 SOC, 0x93, 0x98 ...)"),
        ("vcan_arm_l / _r", "emulated Damiao motors, 7 joints + gripper per arm (send 0x01-0x08, feedback 0x11-0x18, 0x7FF refresh)"),
        ("Arm control", "MoveIt 2 + ros2_control: the real openarm driver (openarm_can) talks to the emulated motors over CAN FD"),
        ("Recording", "ROS 2 bag (MCAP, 31 topics), candump -L of each vcan bus, mission phase table JSON, one observer camera at 1 Hz")]
for i, (a, b) in enumerate(rows):
    y = 0.8 - i * 0.07; f.text(0.04, y, a, fontsize=16, fontweight="bold", color=FG); f.text(0.25, y, b, fontsize=15, color=FG2)
save(f, "c_t1_components.png")

# T2 decoded CAN at one instant + verification
T = 52.0
f = page("Virtual CAN, decoded", f"latest frames at Gazebo t = {T:.1f} s (lift moving up), and a cross-check of the arm decoding against the ROS topic")
ax = f.add_axes([0.03, 0.40, 0.94, 0.46]); ax.axis("off")
lines = []
for b in ["vcan_drive", "vcan_aux", "vcan_bms", "vcan_arm_l"]:
    st = can[b + "__sim"]; i = np.searchsorted(st, T)
    for j in range(i - (8 if b == "vcan_arm_l" else 4), i):
        lines.append(f"{st[j]:8.3f}  {b:<10} {int(can[b + '__id'][j]):>8X}  {can[b + '__data'][j].decode():<19} {cd.decode(b, int(can[b + '__id'][j]), can[b + '__data'][j])}")
ax.text(0, 1, "Gazebo s   bus        ID        raw data            decoded\n" + "\n".join(lines), va="top", family="DejaVu Sans Mono", fontsize=11.3, linespacing=1.33)
# verification over the whole run
a = z["arm__joint_states"]; ids = can["vcan_arm_l__id"]; st = can["vcan_arm_l__sim"]; dt = can["vcan_arm_l__data"]
dq, dtau, n = [], [], 0
for j in range(1, 8):
    sel = np.where(ids == 0x10 + j)[0][::500]; n += len(sel)
    dec = np.array([cd.damiao_fb(0x10 + j, cd.payload(dt[i])) for i in sel])
    dq.append(np.nanmax(np.abs(dec[:, 2] - np.interp(st[sel], a[:, 0], a[:, j])))); dtau.append(np.nanmax(np.abs(dec[:, 4] - np.interp(st[sel], a[:, 0], a[:, 7 + j]))))
VER = (n, max(dq), max(dtau))
ax2 = f.add_axes([0.06, 0.07, 0.40, 0.25])
sel = np.where(ids == 0x14)[0][::200]; dec = np.array([cd.damiao_fb(0x14, cd.payload(dt[i])) for i in sel])
ax2.plot(a[::40, 0], a[::40, 4], color=C[0], label="/arm/joint_states left j4"); ax2.plot(st[sel], dec[:, 2], ls="none", marker=".", ms=4, color=C[3], label="decoded from CAN ID 0x14")
ax2.set_xlabel("Gazebo time (s)"); ax2.set_ylabel("rad"); ax2.legend(loc="upper left"); ax2.set_title("left j4 position: ROS topic vs decoded CAN feedback", loc="left", fontsize=13)
f.text(0.52, 0.28, f"Cross-check over the run ({VER[0]} sampled feedback frames, j1..j7 of vcan_arm_l):", fontsize=14, color=FG)
f.text(0.52, 0.23, f"max |decoded position - /arm/joint_states position| = {VER[1] * 1000:.1f} mrad", fontsize=14, color=FG2)
f.text(0.52, 0.19, f"max |decoded torque - /arm/joint_states effort| = {VER[2]:.3f} N m", fontsize=14, color=FG2)
f.text(0.52, 0.14, "(16-bit position, 12-bit torque quantisation and the host-to-Gazebo time mapping\n are within these differences.) Bus load per host second: arm buses about 3340 frames/s,\n drive 104, lift 16, BMS 6.", fontsize=12.5, color=MUTED, va="top")
save(f, "c_t2_can.png")

# T3 controller traces
f = page("Controller traces", "command and feedback through the emulated devices, from the recorded topics")
lift = z["sil__lift__status"]; lj = z["sil__gz__joint_states"]; lc = z["sil__lift__command"]; pm = z["sil__lift__permit"]
ax = f.add_axes([0.05, 0.50, 0.42, 0.33]); ax.plot(lift[:, 0], lift[:, 1], color=C[0], label="CTL-017 emulator position (/sil/lift/status)")
ax.plot(lj[::5, 0], lj[::5, 1] * 1000, color=C[3], ls="--", lw=1.5, label="Gazebo lift_joint")
for t, v in lc: ax.axvline(t, color=C[4], lw=1); ax.text(t + 0.3, 70, f"cmd {int(v)}", color=C[4], fontsize=11)
ax.set_xlim(38, 66); ax.set_ylabel("mm"); ax.set_xlabel("Gazebo time (s)"); ax.legend(loc="upper left", fontsize=10); ax.set_title("Lift: /sil/lift/command (magenta), emulator report, physics", loc="left", fontsize=14)
E = x["err"]; ax = f.add_axes([0.55, 0.50, 0.42, 0.33])
for k, j in enumerate((1, 2, 4)): ax.plot(E[:, 0], E[:, j] * 1000, color=C[k], lw=1.4, label=f"j{j}")
ax.set_xlim(60, 116); ax.set_ylabel("mrad"); ax.set_xlabel("Gazebo time (s)"); ax.legend(loc="upper left", ncol=3)
ax.set_title("left_arm_controller tracking error (controller_state)", loc="left", fontsize=14)
d = z["sil__drive__status"]; ax = f.add_axes([0.05, 0.08, 0.42, 0.30])
ax.plot(d[:, 0], d[:, 2], color=C[0], lw=1.4, label="target rpm L sent over vcan_drive"); ax.plot(d[:, 0], d[:, 4] * 60 / (2 * np.pi), color=C[3], lw=1.4, label="wheel L speed (rpm) from Gazebo")
ax.set_xlim(7, 42); ax.set_xlabel("Gazebo time (s)"); ax.set_ylabel("rpm"); ax.legend(loc="upper right", fontsize=10); ax.set_title("Drive: command vs wheel (/sil/drive/status)", loc="left", fontsize=14)
g = z["left_arm_gravity_controller__commands"]; a = z["arm__joint_states"]; ax = f.add_axes([0.55, 0.08, 0.42, 0.30])
ax.plot(a[::20, 0], a[::20, 11], color=C[1], lw=1.4, label="j4 effort (/arm/joint_states)"); ax.plot(g[::4, 0], g[::4, 4], color=C[1], ls="--", lw=1.4, label="j4 gravity feed-forward command")
ax.set_xlim(60, 116); ax.set_xlabel("Gazebo time (s)"); ax.set_ylabel("N m"); ax.legend(loc="upper left", fontsize=10); ax.set_title("Arm torque: reported effort vs feed-forward", loc="left", fontsize=14)
save(f, "c_t3_controllers.png")

# T4 physics measurements
f = page("Measurements in the phase table", "measured quantities with their limits, as written by the mission into baseline1_mission.json; each entry names its source in the JSON")
ax = f.add_axes([0.03, 0.06, 0.94, 0.80]); ax.axis("off")
want = ["spawn_position_error", "nav_ground_truth_position_error", "nav_ground_truth_yaw_error", "nav_odometry_minus_ground_truth",
        "station_reach_position_error", "station_reach_yaw_error", "stop_base_speed", "authorize_motion_without_permit",
        "lift_stop_1_gazebo_position", "lift_stop_1_joint_state_position", "lift_stop_0_gazebo_position", "finger_open_position",
        "grasp_reach_cartesian_fraction", "item_pushed_before_closing", "finger_position_on_item", "item_height_gain_after_lift",
        "item_height_drift_while_held", "retreat_left_distance_from_park", "retreat_along_heading"]
Q = {q["name"]: (p["phase"], q) for p in M["phases"] for q in p["quantities"]}
txt = [f"{'phase':<15} {'quantity':<34} {'value':>10} {'unit':<6} {'limits':<20} ok"]
for n in want:
    ph, q = Q[n]; txt.append(f"{ph:<15} {n:<34} {q['value']:>10.4g} {q['unit'][:6]:<6} {format(q['low'], '.4g') + ' to ' + format(q['high'], '.4g'):<20} {'yes' if q['ok'] else 'NO'}")
ax.text(0, 1, "\n".join(txt), va="top", family="DejaVu Sans Mono", fontsize=15, linespacing=1.45)
save(f, "c_t4_physics.png")

# T5 clock behaviour
f = page("Clock behaviour", "Gazebo time drives the device emulators (driver_clock: gazebo); Gazebo time advanced at about 0.21x host time in this run")
tr = M["trace"]; w = np.array([t["wall"] for t in tr]); s = np.array([t["sim"] for t in tr]); r = z["build1__rtf_stats"]
ax = f.add_axes([0.05, 0.47, 0.42, 0.36]); ax.plot(w - w[0], s, color=C[0]); ax.set_xlabel("host time since the first trace sample (s)"); ax.set_ylabel("Gazebo time (s)")
ax.set_title("Gazebo time against host time (mission trace)", loc="left", fontsize=14)
ax = f.add_axes([0.55, 0.47, 0.42, 0.36]); ax.plot(r[:, 0], r[:, 1], color=C[2], lw=1.2, label="rtf_last"); ax.plot(r[:, 0], r[:, 2], color=C[3], lw=1.6, label="rtf_mean")
ax.set_ylim(0, 0.4); ax.set_xlabel("Gazebo time (s)"); ax.legend(loc="upper right"); ax.set_title("Real-time factor (/build1/rtf_stats)", loc="left", fontsize=14)
K = x["clk"]; cnt = {b: len(can[b + "__wall"]) for b in ["vcan_arm_l", "vcan_drive", "vcan_aux", "vcan_bms"]}
dur_s = s[-1] - s[0]; dur_w = w[-1] - w[0]
f.text(0.05, 0.36, "Run clock facts (from the recording)", fontsize=16, fontweight="bold")
facts = [f"Gazebo {s[0]:.1f} to {s[-1]:.1f} s ({dur_s:.1f} s) took {dur_w:.0f} s of host time: mean RTF {dur_s / dur_w:.2f}",
         "physics step 1 ms; arm emulator rate 1000 Hz, driver rate 500 Hz (/build1/run_info)",
         f"lift emulator sim_clock at the end: stalls {int(K[-1, 2])}, jumps {int(K[-1, 3])}, resets {int(K[-1, 4])} (/sil/lift/emulator_state, stall_bound_s 1.0)",
         f"CAN frames are counted in host time: vcan_arm_l {cnt['vcan_arm_l'] / dur_w:.0f}/s host = {cnt['vcan_arm_l'] / dur_s:.0f} per Gazebo s; vcan_drive {cnt['vcan_drive'] / dur_w:.0f}/s host",
         "candump stamps are host time; this video maps them to Gazebo time through the mission trace (0.5 s Gazebo samples)",
         "the observer camera renders at 1 Hz of Gazebo time; its frames are matched to t = 8 + k s (fit to ground-truth motion)"]
for i, t in enumerate(facts): f.text(0.05, 0.31 - i * 0.045, t, fontsize=13.5, color=FG2)
save(f, "c_t5_clock.png")

# T6 limitations and evidence
f = page("Limitations and evidence references", "what this video does not show, and where every number comes from")
lim = ["Simulation only: not hardware, physical acceptance, perception or safety validation.",
       "Simulation-oracle profile: Nav2 planning scene and object / grasp poses come from Gazebo ground truth, not from perception.",
       "The STM32 base controller is a host stand-in (stm32: host), not the firmware and not Renode; it holds no safety function.",
       "Drive, lift, BMS and arm motors are host emulators on virtual CAN with assumed power-up states, thresholds and latency.",
       "No command arbiter: collision_monitor is the single /cmd_vel writer in these runs.",
       "Observer camera: 480 x 270 at 1 Hz of Gazebo time (8 fps file); enlarged for layout, which adds no detail.",
       "The odom > base_footprint TF and contact forces were not recorded in this run; gripper action status was not recorded.",
       "One run (no seed set for Gazebo); repeated runs are not guaranteed identical. Real-time factor about 0.2."]
for i, t in enumerate(lim): f.text(0.04, 0.83 - i * 0.05, "-  " + t, fontsize=14.5, color=FG2)
f.text(0.04, 0.38, "Evidence", fontsize=18, fontweight="bold")
ev = ["Run: https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/44309791416 (artifact baseline1-acceptance-attempt1)",
      "Phase table: build1_logs/baseline1_mission.json; bag: build1_logs/bag (MCAP); CAN: build1_logs/record_candump_vcan_*.log",
      "Commits (pins.txt): openamr-upperbody-sw d63e9ed, openamr-platform-sw 931e631, openarm_can bfd4e52, openarm_ros2 4e837e1",
      "Baseline 1 release: sim-baseline1-2026-11-18 (runs 44309791416, 44309804809, 44309817813, 8/8 each)",
      "Rendering scripts: openamr-upperbody-sw branch diag/sim-videos, tools/video/ (this video is regenerated from the artifact)"]
for i, t in enumerate(ev): f.text(0.04, 0.33 - i * 0.05, t, fontsize=13.5, color=FG2)
save(f, "c_t6_limits.png")
print("verification", VER)
