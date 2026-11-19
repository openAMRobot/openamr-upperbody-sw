# SPDX-License-Identifier: MIT
"""Video B (two-minute system demonstration) of grocery run 44299879412. Left: the recorded store camera. Right: panels
drawn per frame from the recorded bag, candump logs and report JSON. Reproduce: see scripts/README in this branch.
    render_b2.py still SECTION VIDEO_T | render_b2.py section N"""
import sys, json, glob, subprocess, math, re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrow
from PIL import Image
sys.path.insert(0, __file__.rsplit("/", 1)[0]); import candecode as cd

LOG = "in_gd/ws/build1_logs"; FPS = 30; W, H = 1920, 1080
SRC = "openamr-upperbody-sw run 44299879412, demo/grocery-dark-store 6170e6a (artifact grocery-demo-attempt1)"
BG, FG, FG2, MUTED, GRID = "#1a1a19", "#f0efec", "#c3c2b7", "#8d8c86", "#2e2e2c"
C = ["#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#0ca30c", "#9085e9", "#e66767"]
GOOD, BAD = "#0ca30c", "#e66767"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 12, "text.color": FG, "axes.edgecolor": "#5a5955",
                     "xtick.color": FG2, "ytick.color": FG2, "axes.facecolor": BG, "figure.facecolor": BG, "axes.labelcolor": FG2,
                     "axes.grid": True, "grid.color": GRID, "lines.linewidth": 2, "legend.frameon": False, "legend.fontsize": 10.5,
                     "xtick.labelsize": 10.5, "ytick.labelsize": 10.5})
REP = json.load(open(f"{LOG}/grocery_demo_report.json")); PH = REP["phases"]
frames = sorted(glob.glob("gfr/f*.png")); NF = len(frames)
def srt_s(x): h, m, s = x.split(":"); return int(h) * 3600 + int(m) * 60 + float(s.replace(",", "."))
srt = {}
for blk in open(f"{LOG}/grocery_demo.srt").read().strip().split("\n\n"):
    ln = blk.split("\n"); a, b = ln[1].split(" --> "); srt[ln[2].split(":")[0]] = (round(srt_s(a) * 10), round(srt_s(b) * 10))
kf, ks = [], []
for p in PH:
    if p["phase"] in srt: i0, i1 = srt[p["phase"]]; kf += [i0, i1 - 1]; ks += [p["sim_start"], p["sim_end"]]
o = np.argsort(kf); kf, ks = np.array(kf, float)[o], np.array(ks)[o]
def f2sim(f): return float(np.interp(f, kf, ks))
z = np.load("gd.npz", allow_pickle=True); z2 = np.load("gd2.npz"); can = np.load("gd_can.npz")
gt = z["build1__ground_truth__odom"]; odo = z["sil__base__odom"]; amcl = z["amcl_pose"]
item = z["build1__ground_truth__item__pose"]; tote = z["build1__ground_truth__tote__pose"]
plan_t, plan_xy = z["plan_t"], z["plan_xy"]; scan_t, scan_meta, scan_r = z["scan_t"], z["scan_meta"], z["scan_r"]
events = [(float(m[2].split()[0]), re.sub(r", behavior tree .*", "", " ".join(m[2].split()[1:]))) for m in z["msgs"] if m[1] == "/grocery_demo/events"]
cvn, cvs, cv = z2["cmd_vel_nav"], z2["cmd_vel_smoothed"], z2["cmd_vel"]
lift = z2["sil__lift__status"]; lj = z2["sil__gz__joint_states"]; arm = z2["arm__joint_states"]; fing = z2["build1__ground_truth__finger_joint_states"]
drv = z2["sil__drive__status"]; bms = z2["sil__bms__status"]; rtf = z2["build1__rtf_stats"]; cont = z2["contacts"]
lcmd = z2["sil__lift__command"]; perm = z2["sil__lift__permit"]; cerr = z2["left_arm_controller__controller_state"]
STATES = json.load(open(f"{LOG}/grocery_states.json"))
def binmax(a, dt=0.05):
    k = np.floor(a[:, 0] / dt); u, idx = np.unique(k, return_index=True)
    return np.column_stack([u * dt] + [np.maximum.reduceat(a[:, j], idx) for j in range(1, a.shape[1])])
cont = binmax(cont)
def ds(a, hz=20):
    keep = np.concatenate([[True], np.diff(np.floor(a[:, 0] * hz)) > 0]); return a[keep]
arm = ds(arm, 25); lj = ds(lj); fing = ds(fing); cvn, cvs, cv = ds(cvn), ds(cvs), ds(cv)
gxy = lambda t: (np.interp(t, gt[:, 0], gt[:, 1]), np.interp(t, gt[:, 0], gt[:, 2]))
oerr = np.column_stack([odo[:, 0], np.hypot(odo[:, 1] - np.interp(odo[:, 0], gt[:, 0], gt[:, 1]), odo[:, 2] - np.interp(odo[:, 0], gt[:, 0], gt[:, 2]))])
aerr = np.column_stack([amcl[:, 0], np.hypot(amcl[:, 1] - np.interp(amcl[:, 0], gt[:, 0], gt[:, 1]), amcl[:, 2] - np.interp(amcl[:, 0], gt[:, 0], gt[:, 2]))])
oerr = ds(oerr, 10)
# LiDAR returns placed with the ground-truth pose (lidar_link 0.33 m ahead of base_link, yaw pi: recorded /tf_static)
EXT = (-4.6, 5.0, -4.2, 4.2); RES = 0.05; nx, ny = int((EXT[1] - EXT[0]) / RES), int((EXT[3] - EXT[2]) / RES)
cum = []
gyaw = np.unwrap(gt[:, 3])
for k in range(len(scan_t)):
    t = scan_t[k]; a0, da, rmax = scan_meta[k]; r = np.asarray(scan_r[k], dtype=float)
    x, y = gxy(t); th = float(np.interp(t, gt[:, 0], gyaw))
    ok = np.isfinite(r) & (r < rmax * 0.99) & (r > 0.05); ang = (a0 + da * np.arange(len(r)))[ok]; rr = r[ok]
    lx, ly = x + 0.33 * math.cos(th), y + 0.33 * math.sin(th)
    px, py = lx + rr * np.cos(th + math.pi + ang), ly + rr * np.sin(th + math.pi + ang)
    ix = ((px - EXT[0]) / RES).astype(int); iy = ((py - EXT[2]) / RES).astype(int); m = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    cum.append((ix[m], iy[m]))
def upto(a, t): return a[: max(np.searchsorted(a[:, 0], t, side="right"), 1)]
def at(a, t): return upto(a, t)[-1]
def win(a, t, w): i0 = np.searchsorted(a[:, 0], t - w); return a[i0: max(np.searchsorted(a[:, 0], t, side="right"), i0 + 1)]

# sections: (title, subtitle, video seconds, source frame range or None for a card)
SEC = [("Robot, warehouse and task", "a mobile base with a lift and two OpenArm arms picks one product in a simulated dark store", 13, (0, 26)),
       ("Navigation: Nav2, localisation and LiDAR", "Nav2 plans and drives to the shelf; AMCL localises on the LiDAR scan; Gazebo ground truth for comparison", 25, (26, 73)),
       ("ROS 2 commands, virtual CAN and controller feedback", "a ROS 2 lift command becomes CANopen frames to the CTL-017 lift emulator, which reports its position back over CAN", 25, (73, 121)),
       ("Lift, MoveIt, arms, grasp and contacts", "MoveIt 2 plans the left arm; the Damiao motor emulators follow over virtual CAN; Gazebo reports the contacts", 25, (121, 191)),
       ("Controller states and synchronised telemetry", "drive to packing and place, played at about 6x: device states over CAN, and the odometry guard that failed", 15, (191, 379)),
       ("Outcome", "return to parking, about 8x; the phase checks of the run", 5, (379, NF))]

fig = plt.figure(figsize=(W / 100, H / 100), dpi=100)
def ax_px(x0, y0, w, h, **k): return fig.add_axes([x0 / W, y0 / H, w / W, h / H], **k)
def ptitle(ax, title, src):
    ax.set_title(title, loc="left", fontsize=13.5, fontweight="bold", color=FG, pad=21)
    ax.text(0, 1.012, src, transform=ax.transAxes, fontsize=10, color=FG2, va="bottom")
fig.text(20 / W, 1053 / H, "Grocery warehouse demonstration (simulation)", fontsize=21, fontweight="bold", va="center")
fig.text(20 / W, 1020 / H, "Gazebo simulation, simulation-oracle profile (pose_source: ground_truth, object_pose_source: ground_truth)."
         " Not hardware, perception or safety validation.", fontsize=12.5, color=FG2, va="center")
hdr = fig.text(1900 / W, 1053 / H, "", fontsize=15, ha="right", va="center", family="DejaVu Sans Mono")
spd = fig.text(1900 / W, 1020 / H, "", fontsize=12.5, ha="right", va="center", color=FG2)
axi = ax_px(20, 330, 1200, 675); axi.axis("off"); img = axi.imshow(np.asarray(Image.open(frames[0])), aspect="auto", interpolation="bilinear")
axi.text(0.005, 0.012, "RECORDED store camera: 960 x 540, 2 frames per Gazebo second, shown at 1.25x (no added detail)",
         transform=axi.transAxes, ha="left", va="bottom", fontsize=10.5, color="#0b0b0b", bbox=dict(boxstyle="square,pad=0.3", fc="#f0efec", ec="none", alpha=0.85))
sec_t = fig.text(20 / W, 296 / H, "", fontsize=22, fontweight="bold", va="center")
sec_s = fig.text(20 / W, 262 / H, "", fontsize=13.5, color=FG2, va="center")
fail_t = fig.text(20 / W, 232 / H, "", fontsize=13, color=BAD, va="center")
# phase strip, 2 rows x 6
axp = ax_px(20, 60, 1200, 140); axp.axis("off"); axp.set_xlim(0, 6); axp.set_ylim(0, 2)
cells = []
for i, p in enumerate(PH):
    cx, cy = i % 6, 1 - i // 6
    r = Rectangle((cx + 0.03, cy + 0.06), 0.94, 0.88, fc="#232321", ec="#5a5955", lw=1); axp.add_patch(r)
    axp.text(cx + 0.5, cy + 0.64, p["phase"], ha="center", va="center", fontsize=11.5, fontweight="bold")
    v = axp.text(cx + 0.5, cy + 0.3, "", ha="center", va="center", fontsize=11.5); cells.append((r, v, p))
fig.text(20 / W, 204 / H, "phase checks of the run (grocery_demo_report.json)", fontsize=10.5, color=MUTED)
evt = fig.text(20 / W, 30 / H, "", fontsize=11, color=FG2, family="DejaVu Sans Mono", va="center")
fig.text(1900 / W, 8 / H, "Source: " + SRC + ". Panels are drawn from the recording; nothing is re-simulated.", fontsize=9.5, color=MUTED, ha="right")
RX, RW = 1270, 630
groups = {}
def grp(name, *artists): groups.setdefault(name, []).extend(artists)

# --- S0 system + map, S1 nav: replay map
axm = ax_px(RX + 40, 600, RW - 60, 345)
gim = axm.imshow(np.zeros((ny, nx)), extent=EXT, origin="lower", cmap="gray", vmin=0, vmax=1, interpolation="nearest", alpha=0.9)
ln_plan, = axm.plot([], [], color=C[6], lw=1.8, ls="--", label="Nav2 /plan (latest)")
ln_gt, = axm.plot([], [], color=C[0], lw=2.2, label="path, Gazebo truth")
ln_am, = axm.plot([], [], color=C[1], marker="o", ms=3.5, lw=0, label="/amcl_pose")
ln_od, = axm.plot([], [], color=C[2], lw=1.4, label="wheel odometry /sil/base/odom")
dot_it, = axm.plot([], [], marker="s", ms=8, color=C[3], ls="none", mec=BG, label="product (truth)")
arrow = [None]
axm.set_xlim(EXT[0], EXT[1]); axm.set_ylim(EXT[2], EXT[3]); axm.set_aspect("equal"); axm.grid(False)
ptitle(axm, "Top view REPLAY (drawn, not a camera)", "white: LiDAR /scan_filtered placed with the ground-truth pose; metres")
leg_m = axm.legend(loc="upper left", bbox_to_anchor=(-0.05, -0.06), ncol=2, fontsize=10)
grp("map", axm)
# S0 text block
sys_txt = fig.text(RX / W, 500 / H, "", fontsize=11.5, va="top", linespacing=1.45, color=FG2)
grp("sys", sys_txt)
# S1 nav error trace
axe = ax_px(RX + 40, 90, RW - 60, 250); ptitle(axe, "Pose error against Gazebo truth", "AMCL (/amcl_pose) and wheel odometry (/sil/base/odom), m")
ln_ea, = axe.plot([], [], color=C[1], label="AMCL - truth"); ln_eo, = axe.plot([], [], color=C[2], label="odometry - truth")
axe.set_ylim(0, 0.2); axe.legend(loc="upper left"); axe.set_xlabel("Gazebo time (s)"); grp("navx", axe)
# S2 command chain / CAN / lift feedback
axl = ax_px(RX + 40, 690, RW - 60, 270); ptitle(axl, "Lift: command, CAN frames, feedback", "magenta: /sil/lift/command; TPDO1 position from vcan_aux vs Gazebo, mm")
ln_lc, = axl.plot([], [], color=C[0], label="position in TPDO1 0x190 (CAN)"); ln_lp, = axl.plot([], [], color=C[3], ls="--", lw=1.5, label="Gazebo lift_joint")
axl.set_ylim(-10, 200); axl.legend(loc="upper left"); axl.set_xlim(40, 68); grp("lift", axl)
lc_marks = []
axcv = ax_px(RX + 40, 400, RW - 60, 210); ptitle(axcv, "Base velocity command chain (drive to the shelf)", "/cmd_vel_nav -> /cmd_vel_smoothed -> collision_monitor -> /cmd_vel, m/s")
ln_c1, = axcv.plot([], [], color=C[6], lw=1.4, label="/cmd_vel_nav"); ln_c2, = axcv.plot([], [], color=C[2], lw=1.4, label="/cmd_vel_smoothed")
ln_c3, = axcv.plot([], [], color=C[0], lw=2, label="/cmd_vel"); axcv.set_ylim(-0.35, 0.45); axcv.legend(loc="upper left", ncol=3, fontsize=9.5)
grp("cmd", axcv)
can_txt = fig.text(RX / W, 330 / H, "", fontsize=9.8, va="top", family="DejaVu Sans Mono", linespacing=1.4)
can_hd = fig.text(RX / W, 352 / H, "", fontsize=13.5, fontweight="bold", va="center")
grp("can", can_txt, can_hd)
# S3 arm / contacts
axa = ax_px(RX + 40, 760, RW - 60, 200); ptitle(axa, "Left arm joints (/arm/joint_states, rad)", "openarm driver, reading the Damiao motor emulators over vcan_arm_l")
ln_a = {j: axa.plot([], [], color=C[k], label=f"j{j}")[0] for k, j in enumerate((1, 2, 4))}; axa.set_ylim(-1.4, 1.9); axa.legend(loc="upper left", ncol=3)
axf = ax_px(RX + 40, 470, RW - 60, 200); ptitle(axf, "Gripper and contact force", "finger: ground truth (rad x10); force: /build1/contacts/world_item, N")
ln_fg, = axf.plot([], [], color=C[3], lw=1.6, label="finger joint1 x10 (rad)"); ln_ff, = axf.plot([], [], color=C[0], lw=1.2, label="item-finger force")
ln_ft, = axf.plot([], [], color=C[2], lw=1.2, label="item-support force"); axf.set_ylim(0, 25); axf.legend(loc="upper left", ncol=3, fontsize=9.5)
clip_t = axf.text(0.99, 0.95, "", transform=axf.transAxes, ha="right", va="top", fontsize=9, color=MUTED)
pick_txt = fig.text(RX / W, 380 / H, "", fontsize=11, va="top", family="DejaVu Sans Mono", linespacing=1.5)
grp("arm", axa, axf, pick_txt, clip_t)
# S4 telemetry
st_txt = fig.text(RX / W, 975 / H, "", fontsize=11.2, va="top", family="DejaVu Sans Mono", linespacing=1.42)
axo = ax_px(RX + 40, 110, RW - 60, 230); ptitle(axo, "Odometry guard (nav_to_packing)", "| /sil/base/odom - Gazebo truth |, m; guard 0.56 m")
ln_og, = axo.plot([], [], color=C[2]); axo.axhline(0.56, color=BAD, lw=1.2, ls="--"); axo.text(102, 0.6, "guard 0.56 m", color=BAD, fontsize=10)
axo.set_xlim(100, 184); axo.set_ylim(0, 1.0); axo.set_xlabel("Gazebo time (s)")
axw = ax_px(RX + 40, 420, RW - 60, 190); ptitle(axw, "Drive: commanded vs measured wheel speed", "/sil/drive/status: target rpm sent over vcan_drive vs wheel rad/s")
ln_tr, = axw.plot([], [], color=C[0], lw=1.4, label="target L (rpm / 10)"); ln_wr, = axw.plot([], [], color=C[3], lw=1.4, label="wheel L (rad/s)")
axw.set_xlim(100, 184); axw.set_ylim(-3.5, 3.5); axw.legend(loc="upper left", ncol=2, fontsize=9.5)
grp("tel", st_txt, axo, axw)
# S5 outcome
out_txt = fig.text(RX / W, 975 / H, "", fontsize=13, va="top", linespacing=1.55)
grp("out", out_txt)
SHOW = [("map", "sys"), ("map", "navx"), ("lift", "cmd", "can"), ("arm",), ("tel",), ("out",)]
for g in groups.values():
    for a in g: a.set_visible(False)

state = {"k": 0, "grid": np.zeros((ny, nx), np.float32), "sec": -1}
def fail_line(t):
    fs = [p for p in PH if p["verdict"] == "FAIL" and t >= p["sim_end"]]
    txt = {"nav_to_shelf": "nav_to_shelf FAIL: parked 49.7 mm from the shelf station goal, limit 40 mm (station alignment off in this run)",
           "nav_to_packing": "nav_to_packing FAIL: odometry guard, odometry minus truth 0.907 m against 0.56 m (attributed to Gazebo wheel collision slip)"}
    return txt[fs[-1]["phase"]] if fs else ""
def recent_frames(t, buses, n_each=2):
    rows = []
    for b in buses:
        st = can[b + "__sim"]; i = np.searchsorted(st, t, side="right")
        for j in range(max(i - n_each, 0), i): rows.append((st[j], b, int(can[b + "__id"][j]), can[b + "__data"][j]))
    rows.sort(); return rows
def draw(si, f, rate_x):
    if si != state["sec"]:
        for g in groups.values():
            for a in g: a.set_visible(False)
        for name in SHOW[si]:
            for a in groups[name]: a.set_visible(True)
        state["sec"] = si
    fi = int(min(max(f, 0), NF - 1)); t = f2sim(f)
    img.set_data(np.asarray(Image.open(frames[fi])))
    r = at(rtf, t); hdr.set_text(f"Gazebo time {t:6.1f} s   RTF {r[1]:.2f}")
    spd.set_text(f"section {si + 1}/6, playback about {rate_x:.1f}x Gazebo time")
    sec_t.set_text(f"{si + 1}/6  {SEC[si][0]}"); sec_s.set_text(SEC[si][1]); fail_t.set_text(fail_line(t))
    for rr, v, p in cells:
        if t >= p["sim_end"]:
            ok = p["verdict"] == "PASS"; v.set_text("✓ PASS" if ok else "✗ FAIL"); v.set_color(GOOD if ok else BAD)
            rr.set_fc("#16301a" if ok else "#3a1c1c"); rr.set_ec(GOOD if ok else BAD)
        elif t >= p["sim_start"]: v.set_text("running"); v.set_color(C[0]); rr.set_fc("#1d2f47"); rr.set_ec(C[0])
        else: v.set_text("pending"); v.set_color(MUTED); rr.set_fc("#232321"); rr.set_ec("#5a5955")
    ev = [e for e in events if e[0] <= t]; evt.set_text(("latest mission event (/grocery_demo/events): " + f"{ev[-1][0]:.2f} s  {ev[-1][1]}")[:140] if ev else "")
    while state["k"] < len(cum) and scan_t[state["k"]] <= t:
        ix, iy = cum[state["k"]]; g = state["grid"]; g[iy, ix] = 1; g[np.minimum(iy + 1, ny - 1), ix] = 1; g[iy, np.minimum(ix + 1, nx - 1)] = 1; state["k"] += 1
    if "map" in SHOW[si]:
        gim.set_data(state["grid"]); g = upto(gt, t); ln_gt.set_data(g[:, 1], g[:, 2])
        a = upto(amcl, t); ln_am.set_data(a[:, 1], a[:, 2]); o = upto(odo, t)[::10]; ln_od.set_data(o[:, 1], o[:, 2])
        if arrow[0] is not None: arrow[0].remove()
        th = g[-1, 3]; arrow[0] = axm.add_patch(FancyArrow(g[-1, 1], g[-1, 2], 0.45 * math.cos(th), 0.45 * math.sin(th), width=0.14,
                                                           head_width=0.36, head_length=0.25, color=C[1], zorder=6))
        pi = np.searchsorted(plan_t, t, side="right") - 1
        if pi >= 0 and len(plan_xy[pi]): ln_plan.set_data(plan_xy[pi][:, 0], plan_xy[pi][:, 1])
        it = at(item, t); dot_it.set_data([it[1]], [it[2]])
    if si == 0:
        sys_txt.set_text(
            "What runs in this simulation\n"
            "Physics: Gazebo Harmonic, DART engine (robot, store, contacts)\n"
            "ROS 2 Jazzy: Nav2 + AMCL, MoveIt 2, ros2_control, openarm driver\n"
            "Base commands: Nav2 -> velocity smoother -> collision_monitor,\n"
            "  the single /cmd_vel writer (no command arbiter in this run)\n"
            "Virtual CAN (vcan) device emulators:\n"
            "  vcan_drive  ZLAC8015D drive emulator, CANopen node 1\n"
            "  vcan_aux    CTL-017 lift controller, host-side emulator, node 0x10\n"
            "  vcan_bms    Daly BMS emulator\n"
            "  vcan_arm_l/r  emulated Damiao motors, 8 per arm\n"
            "STM32 base controller: host stand-in (stm32: host),\n  not firmware in Renode\n"
            "Task: pick one product from a shelf tray, place it in the order tote")
    if si == 1:
        e = upto(aerr, t); ln_ea.set_data(e[:, 0], e[:, 1]); e2 = upto(oerr, t); ln_eo.set_data(e2[:, 0], e2[:, 1]); axe.set_xlim(5, max(45, t + 1))
    if si == 2:
        L = upto(lift, t); ln_lc.set_data(L[:, 0], L[:, 1]); P = upto(lj, t); ln_lp.set_data(P[:, 0], P[:, 1] * 1000)
        while len(lc_marks) < np.searchsorted(lcmd[:, 0], t, side="right"):
            tc, v = lcmd[len(lc_marks)]; lc_marks.append(axl.axvline(tc, color=C[4], lw=1)); axl.text(tc + 0.3, 120 - 25 * len(lc_marks), f"cmd {int(v)}", fontsize=10, color=C[4])
        for a_, ln in ((cvn, ln_c1), (cvs, ln_c2), (cv, ln_c3)):
            w_ = upto(a_, min(t, 45.0)); ln.set_data(w_[:, 0], w_[:, 1])
        axcv.set_xlim(15, 45)
        can_hd.set_text("Decoded CAN frames (2 latest per bus, Gazebo time)")
        can_txt.set_text("\n".join(f"{s:7.2f} {b:<10} {i:>8X}  {cd.decode(b, i, d)}"[:84] for s, b, i, d in recent_frames(t, ["vcan_drive", "vcan_aux", "vcan_bms", "vcan_arm_l"])))
    if si == 3:
        A = upto(arm, t)
        for j in (1, 2, 4): ln_a[j].set_data(A[:, 0], A[:, j])
        F = upto(fing, t); ln_fg.set_data(F[:, 0], F[:, 1] * 10); K = upto(cont, t); ln_ff.set_data(K[:, 0], K[:, 1]); ln_ft.set_data(K[:, 0], K[:, 2])
        for ax_ in (axa, axf): ax_.set_xlim(64, 102)
        mx = K[:, 1:].max() if len(K) else 0
        clip_t.set_text(f"peaks above 25 N clipped (max so far {mx:.0f} N)" if mx > 25 else "")
        pk = [e for e in events if e[0] <= t and e[1].startswith(("pick", "phase", "backup"))][-4:]
        L = at(lift, t); E = at(cerr, t)
        pick_txt.set_text("MoveIt / mission steps:\n" + "\n".join(f"  {a_:7.2f} s  {b_}" for a_, b_ in pk) +
                          f"\nlift (TPDO1 0x190): {L[1]:.1f} mm\nleft_arm_controller error j1..j4 (mrad): " + " ".join(f"{x * 1000:+.0f}" for x in E[1:5]))
    if si == 4:
        d = at(drv, t); L = at(lift, t); b = at(bms, t)
        cs = [s for s in STATES if s["sim"] <= t]; cs = cs[-1] if cs else STATES[0]
        rows = recent_frames(t, ["vcan_drive", "vcan_aux", "vcan_arm_l"], 1)
        st_txt.set_text(
            "Device and controller states (recorded topics)\n"
            f"drive  ZLAC8015D emu  statusword {int(d[1])}  fault {int(d[6])}\n"
            f"       target rpm L {d[2]:+4.0f} R {d[3]:+4.0f}, wheels {d[4]:+5.2f} {d[5]:+5.2f} rad/s\n"
            f"lift   CTL-017 emu    state {int(L[3])}  {L[1]:5.1f} mm  permit {'on' if L[4] else 'off'}\n"
            f"bms    Daly emu       SOC {b[1]:.1f} %  {b[2]:.1f} V  {b[3]:+.2f} A\n"
            f"ros2_control: {sum(v == 'active' for v in cs['controllers'].values())}/{len(cs['controllers'])} controllers active at {cs['sim']:.1f} s\n"
            + (f"in Gazebo: {', '.join(k for k, v in cs['sim_controllers'].items() if v == 'active')}")[:64] + "\n\n"
            + "Latest decoded frames\n" + "\n".join(f"{s:7.2f} {bb:<10} {i:>4X} {cd.decode(bb, i, dd)}"[:62] for s, bb, i, dd in rows))
        O = upto(oerr, t); ln_og.set_data(O[:, 0], O[:, 1]); D = ds(upto(drv, t), 10); ln_tr.set_data(D[:, 0], D[:, 2] / 10); ln_wr.set_data(D[:, 0], D[:, 4])
    if si == 5:
        n_ok = sum(p["verdict"] == "PASS" for p in PH)
        out_txt.set_text(f"Phase checks: {n_ok} of {len(PH)} passed\n\n"
                         "Failed:\n  nav_to_shelf  parked 49.7 mm from the goal\n     (limit 40 mm; station alignment off)\n"
                         "  nav_to_packing  odometry guard 0.907 m\n     (guard 0.56 m; attributed to wheel slip\n      of the Gazebo wheel collision)\n\n"
                         "Completed: pick, transport, place into the\norder tote, return to parking\n\n"
                         f"path {REP['grocery']['travelled']:.1f} m, {REP['grocery']['sim_seconds']:.0f} s Gazebo time,\n{REP['grocery']['wall_seconds']:.0f} s host time")

def render(si, out):
    a, b = SEC[si][3]; n = int(round(SEC[si][2] * FPS)); rate_src = (b - a) / n      # source frames per output frame
    p = subprocess.Popen(["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS),
                          "-i", "-", "-c:v", "libx264", "-preset", "medium", "-crf", "19", "-pix_fmt", "yuv420p", out], stdin=subprocess.PIPE)
    for i in range(n):
        f = a + i * rate_src; draw(si, f, rate_src * FPS / 2)
        fig.canvas.draw(); p.stdin.write(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].tobytes())
    p.stdin.close(); p.wait()
if __name__ == "__main__":
    if sys.argv[1] == "still":
        si, vt = int(sys.argv[2]), float(sys.argv[3]); a, b = SEC[si][3]; f = a + (b - a) * vt / SEC[si][2]
        for k in range(len(scan_t)):
            if scan_t[k] > f2sim(f): break
        draw(si, f, (b - a) / SEC[si][2] / 2); fig.savefig(f"still_b2_{si}_{vt:04.1f}.png", dpi=100)
    else:
        render(int(sys.argv[2]), f"b2_s{int(sys.argv[2])}.mp4")
