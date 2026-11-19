# SPDX-License-Identifier: MIT
"""Video C renderer: observer frames (left) and panels drawn from the recorded bag, candump and mission JSON (right).
Every drawn signal comes from the recording of run 44309791416. Output: raw RGB frames piped to ffmpeg."""
import sys, json, glob, subprocess
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from PIL import Image
sys.path.insert(0, __file__.rsplit("/", 1)[0]); import candecode as cd

LOG = "in_b1/ws/build1_logs"
FPS = 30; W, H = 1920, 1080
SIM0, SIM1 = 7.40, 115.90          # bag 7.338 to 115.958; panels play at 1x Gazebo time
OBS_T0 = 8.0                       # observer frame k <-> Gazebo t = 8.0 + k s (1 Hz sensor; fit 7.7 to 8.0 s)
BG, FG, FG2, MUTED, GRID = "#1a1a19", "#f0efec", "#c3c2b7", "#8d8c86", "#2e2e2c"
C = ["#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#0ca30c", "#9085e9", "#e66767"]
GOOD = "#0ca30c"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 11, "text.color": FG, "axes.labelcolor": FG2,
                     "axes.edgecolor": "#5a5955", "xtick.color": FG2, "ytick.color": FG2, "axes.facecolor": BG,
                     "figure.facecolor": BG, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
                     "lines.linewidth": 2, "legend.frameon": False, "legend.fontsize": 9.5})

z = np.load("b1.npz", allow_pickle=True)
can = np.load("b1_can.npz")
M = json.load(open(f"{LOG}/baseline1_mission.json"))
phases = [(p["phase"], p["sim_start"], p["sim_end"], p["ran"] and not p["failures"]) for p in M["phases"]]
goal_ev = [e["text"] for e in M["events"] if e["text"].startswith("nav goal work_shelf")][0]
GOAL = (0.0, -2.085, -1.5708)       # parsed below from the event text, asserted equal
_g = goal_ev.split(); assert abs(float(_g[4]) - GOAL[0]) < 1e-3 and abs(float(_g[6]) - GOAL[1]) < 1e-3
obs = [np.asarray(Image.open(f).convert("RGB")) for f in sorted(glob.glob("obs/f*.png"))]

def ds(a, hz=20):
    """Downsample rows of a (time in column 0) to at most hz per Gazebo second for drawing."""
    keep = np.concatenate([[True], np.diff(np.floor(a[:, 0] * hz)) > 0]); return a[keep]

gt = ds(z["build1__ground_truth__odom"]); amcl = z["amcl_pose"]; drv = z["sil__drive__status"]
lift = z["sil__lift__status"]; lj = ds(z["sil__gz__joint_states"]); arm = ds(z["arm__joint_states"], 25)
grav = ds(z["left_arm_gravity_controller__commands"], 25); fing = ds(z["build1__ground_truth__finger_joint_states"])
item = ds(z["build1__ground_truth__item__pose"]); rtf = z["build1__rtf_stats"]; bms = z["sil__bms__status"]
permit = z["sil__lift__permit"]; plan_t = z["plan_t"]; plan_xy = z["plan_xy"]

BUSES = ["vcan_arm_l", "vcan_arm_r", "vcan_drive", "vcan_aux", "vcan_bms"]
BUSLBL_UNUSED = {"vcan_arm_l": "vcan_arm_l (OpenArm L motors)", "vcan_arm_r": "vcan_arm_r (OpenArm R motors)",
          "vcan_drive": "vcan_drive (ZLAC8015D drive)", "vcan_aux": "vcan_aux (CTL-017 lift)", "vcan_bms": "vcan_bms (Daly BMS)"}
rate = {}
for b in BUSES:
    wt = can[b + "__wall"]; st = can[b + "__sim"]; edges = np.arange(wt[0], wt[-1], 1.0)
    cnt, _ = np.histogram(wt, edges); ctr = edges[:-1] + 0.5
    ok = ~np.isnan(np.interp(ctr, wt, st, left=np.nan, right=np.nan))
    rate[b] = np.column_stack([np.interp(ctr, wt, st)[ok], cnt[ok]])
CANOPEN = ((0x000, 0x000, "NMT"), (0x080, 0x080, "SYNC"), (0x081, 0x0FF, "EMCY"), (0x180, 0x1FF, "TPDO1"),
           (0x200, 0x27F, "RPDO1"), (0x280, 0x2FF, "TPDO2"), (0x300, 0x37F, "RPDO2"), (0x380, 0x3FF, "TPDO3"),
           (0x400, 0x47F, "RPDO3"), (0x480, 0x4FF, "TPDO4"), (0x500, 0x57F, "RPDO4"), (0x580, 0x5FF, "SDO tx"),
           (0x600, 0x67F, "SDO rx"), (0x700, 0x77F, "heartbeat"))
def fclass(bus, i):
    if bus in ("vcan_drive", "vcan_aux"):
        for lo, hi, n in CANOPEN:
            if lo <= i <= hi: return n
        return "other"
    if i > 0x7FF: return "ext. ID"
    return "CAN FD" if bus.startswith("vcan_arm") else "11-bit"

fig = plt.figure(figsize=(W / 100, H / 100), dpi=100)
def ax_px(x0, y0, w, h, **k): return fig.add_axes([x0 / W, y0 / H, w / W, h / H], **k)
def ptitle(ax, title, src):
    ax.set_title(title, loc="left", fontsize=12.5, fontweight="bold", color=FG, pad=19)
    ax.text(0, 1.015, src, transform=ax.transAxes, fontsize=9, color=FG2, va="bottom")

# header
fig.text(20 / W, 1052 / H, "Engineering digital twin explained: Baseline 1 mission, run 44309791416 (8 of 8 phases PASS)",
         fontsize=17, fontweight="bold", va="center")
fig.text(20 / W, 1022 / H, "Gazebo simulation, simulation-oracle profile (pose_source: ground_truth, object_pose_source: ground_truth)."
         "  Simulation only: not hardware, perception or safety validation.", fontsize=12, color=FG2, va="center")
hdr = fig.text(1900 / W, 1052 / H, "", fontsize=13.5, ha="right", va="center", family="DejaVu Sans Mono")

# observer
axo = ax_px(20, 480, 920, 518); axo.axis("off")
im = axo.imshow(obs[0], interpolation="bilinear", aspect="auto")
axo.text(0.01, 0.98, "OBSERVER CAMERA  [Gazebo render]", transform=axo.transAxes, fontsize=11, fontweight="bold",
         va="top", color="#0b0b0b", bbox=dict(boxstyle="square,pad=0.3", fc="#f0efec", ec="none", alpha=0.85))
obs_lbl = axo.text(0.01, 0.015, "", transform=axo.transAxes, fontsize=10, va="bottom", color="#0b0b0b",
                   bbox=dict(boxstyle="square,pad=0.3", fc="#f0efec", ec="none", alpha=0.85))

# phase timeline
axp = ax_px(20, 350, 920, 72); axp.set_xlim(0, 8); axp.set_ylim(0, 1); axp.axis("off")
ptitle(axp, "Mission phases", "phase start, end and verdict: baseline1_mission.json (the mission's own phase table)")
cells = []
for i, (n, a, b, ok) in enumerate(phases):
    r = Rectangle((i + 0.04, 0.05), 0.92, 0.9, fc="#262624", ec="#5a5955", lw=1); axp.add_patch(r)
    t1 = axp.text(i + 0.5, 0.74, n, ha="center", va="center", fontsize=9.5, fontweight="bold")
    axp.text(i + 0.5, 0.47, f"{a:.1f} to {b:.1f} s", ha="center", va="center", fontsize=8.5, color=FG2)
    t3 = axp.text(i + 0.5, 0.2, "pending", ha="center", va="center", fontsize=9.5, color=MUTED)
    cells.append((r, t3, a, b, ok))

# emulator state text + recent CAN frames
axs = ax_px(20, 222, 920, 92); axs.axis("off")
ptitle(axs, "Device emulator states", "/sil/lift/status, /sil/drive/status, /sil/bms/status (recorded topics; devices emulated on the host over virtual CAN)")
st_txt = axs.text(0, 0.98, "", va="top", fontsize=10, family="DejaVu Sans Mono", linespacing=1.35)
axc = ax_px(20, 22, 920, 150); axc.axis("off")
ptitle(axc, "Recent CAN frames, decoded (latest per bus)", "candump -L per vcan bus, host time mapped to Gazebo time; decoders: scripts/candecode.py")
can_txt = axc.text(0, 0.98, "", va="top", fontsize=9.6, family="DejaVu Sans Mono", linespacing=1.3)

# map
axm = ax_px(1000, 650, 300, 320)
ptitle(axm, "Navigation (map frame, m)", "[Nav2, AMCL, CAN odometry, Gazebo truth]")
axm.set_aspect("equal", adjustable="box"); axm.set_xlim(-1.3, 1.3); axm.set_ylim(-2.45, 0.35)
axm.plot(*GOAL[:2], marker="*", ms=16, color=C[3], ls="none", label="Nav2 goal (event log)", zorder=5)
ln_plan, = axm.plot([], [], color=C[6], lw=2, ls="--", label="Nav2 /plan (latest)")
ln_gt, = axm.plot([], [], color=C[0], lw=2.5, label="ground truth (Gazebo)")
ln_amcl, = axm.plot([], [], color=C[1], marker="o", ms=5, lw=1.2, label="/amcl_pose")
ln_odo, = axm.plot([], [], color=C[2], lw=1.6, label="wheel odometry (CAN)")
dot_gt, = axm.plot([], [], marker="o", ms=10, color=C[0], mec=BG, mew=2, ls="none")
axm.legend(loc="lower left", bbox_to_anchor=(1.02, 0.0), fontsize=9, handlelength=1.6)
axm.text(1.04, 1.0, "goal: nav goal event in\nbaseline1_mission.json\nplan: /plan (Nav2)\n"
         "ground truth:\n /build1/ground_truth/odom\nAMCL: /amcl_pose\nodometry: /sil/drive/status\n x, y of the STM32 host\n"
         " stand-in, from wheel\n speeds over vcan_drive", transform=axm.transAxes, va="top", fontsize=8.5, color=FG2)
map_err = axm.text(0.03, 0.97, "", transform=axm.transAxes, va="top", fontsize=9, family="DejaVu Sans Mono",
                   bbox=dict(fc=BG, ec="none", alpha=0.8))

def tax(x0, y0, w, h, title, src, ylab):
    a = ax_px(x0, y0, w, h); ptitle(a, title, src); a.set_xlim(SIM0 - 0.5, SIM1 + 0.5); a.set_ylabel(ylab)
    for n, s, e, _ in phases: a.axvline(s, color="#3a3a37", lw=1, zorder=0)
    cur = a.axvline(SIM0, color=FG, lw=1.2, alpha=0.7); return a, cur

# lift
axl, cl = tax(1580, 650, 320, 320, "Lift position", "CTL-017 host emulator (vcan_aux) vs Gazebo joint", "mm")
ln_lfw, = axl.plot([], [], color=C[0], label="emulator report (/sil/lift/status)")
ln_lph, = axl.plot([], [], color=C[3], lw=1.4, ls="--", label="physics lift_joint (/sil/gz/joint_states)")
axl.set_ylim(-10, 200); axl.legend(loc="upper left", fontsize=8.5)
lift_val = axl.text(0.97, 0.5, "", transform=axl.transAxes, ha="right", fontsize=9.5, family="DejaVu Sans Mono")

# arm positions and torques
JN = {1: "j1", 2: "j2", 4: "j4"}
axa, ca = tax(1000, 370, 410, 195, "Left arm joint positions", "/arm/joint_states (openarm driver, emulated Damiao motors)", "rad")
ln_pos = {j: axa.plot([], [], color=C[k], label=f"left {JN[j]}")[0] for k, j in enumerate((1, 2, 4))}
axa.set_ylim(-1.4, 1.9); axa.legend(loc="upper left", ncol=3, fontsize=8.5)
axt, ct = tax(1490, 370, 410, 195, "Left arm joint torques", "solid: /arm/joint_states effort; dashed: gravity feed-forward cmd", "N m")
ln_eff = {j: axt.plot([], [], color=C[k], lw=1.6, label=f"effort {JN[j]}")[0] for k, j in enumerate((2, 4))}
ln_gff = {j: axt.plot([], [], color=C[k], lw=1.4, ls="--", label=f"gravity ff {JN[j]}")[0] for k, j in enumerate((2, 4))}
axt.set_ylim(-1.5, 5.5); axt.legend(loc="upper left", ncol=2, fontsize=8.5)

# CAN rates
axr, cr = tax(1000, 85, 410, 195, "CAN frame rate per bus", "[virtual CAN]  candump -L, frames per host second, log", "frames/s")
axr.set_yscale("log"); axr.set_ylim(2, 2e4); axr.set_xlabel("Gazebo time (s)")
ln_rate = {b: axr.plot([], [], color=C[k], lw=1.6, label=b)[0] for k, b in enumerate(BUSES)}
axr.legend(loc="upper left", ncol=3, fontsize=8.2)

# gripper and item
axg, cg = tax(1490, 85, 410, 195, "Gripper and grasped item", "[Gazebo truth] ground_truth/finger_joint_states, item/pose", "")
axg.set_ylim(-0.05, 1.0); axg.set_xlabel("Gazebo time (s)")
ln_f, = axg.plot([], [], color=C[0], label="left finger joint1 (rad)")
ln_iz, = axg.plot([], [], color=C[3], label="item height above start (dm)")
axg.legend(loc="upper left", fontsize=8.5)
axg.text(0.03, 0.45, "contact force: not recorded in this run", transform=axg.transAxes, ha="left", va="top",
         fontsize=8.5, color=MUTED)
item_z0 = item[0, 3]

fig.text(1900 / W, 6 / H, "Source: openamr-upperbody-sw run 44309791416, integration/baseline1 d63e9ed, openamr-platform-sw 931e631 (artifact baseline1-acceptance-attempt1). "
         "Panels drawn from the recording at 1x Gazebo time; nothing is re-simulated.",
         ha="right", va="bottom", fontsize=9, color=MUTED)

def upto(a, t): return a[: np.searchsorted(a[:, 0], t, side="right")]
def at(a, t): i = np.searchsorted(a[:, 0], t, side="right") - 1; return a[max(i, 0)]
LSTATE = {0: "init", 1: "homing", 2: "idle?", 3: "idle", 4: "moving", 5: "fault"}

def draw(t):
    k = int(np.clip(round(t - OBS_T0), 0, len(obs) - 1))
    im.set_data(obs[k]); obs_lbl.set_text(f"480 x 270 at 1 frame per Gazebo second, enlarged 1.9x (adds no detail). Frame {k + 1}/{len(obs)}, t = {OBS_T0 + k:.0f} s, held 1 s")
    r = at(rtf, t); hdr.set_text(f"Gazebo time {t:6.1f} s   RTF {r[1]:.2f}   host {r[4] - rtf[0, 4]:4.0f} s")
    for (rect, t3, a, b, ok) in cells:
        if t >= b: rect.set_fc("#16301a"); rect.set_ec(GOOD); t3.set_text("✓ PASS" if ok else "FAIL"); t3.set_color(GOOD if ok else C[7])
        elif t >= a: rect.set_fc("#1d2f47"); rect.set_ec(C[0]); t3.set_text("running"); t3.set_color(C[0])
        else: rect.set_fc("#262624"); rect.set_ec("#5a5955"); t3.set_text("pending"); t3.set_color(MUTED)
    g = upto(gt, t); ln_gt.set_data(g[:, 1], g[:, 2]); dot_gt.set_data([g[-1, 1]], [g[-1, 2]])
    am = upto(amcl, t); ln_amcl.set_data(am[:, 1], am[:, 2])
    od = upto(drv, t); ln_odo.set_data(od[:, 7], od[:, 8])
    pi = np.searchsorted(plan_t, t, side="right") - 1
    if pi >= 0: ln_plan.set_data(plan_xy[pi][:, 0], plan_xy[pi][:, 1])
    a1 = am[-1] if len(am) else None
    o1 = od[-1, 7:9]
    map_err.set_text(f"AMCL - truth {np.hypot(a1[1] - np.interp(a1[0], gt[:, 0], gt[:, 1]), a1[2] - np.interp(a1[0], gt[:, 0], gt[:, 2])) * 1000:5.1f} mm\n"
                     f"odom - truth {np.hypot(o1[0] - g[-1, 1], o1[1] - g[-1, 2]) * 1000:5.1f} mm" if a1 is not None else "")
    lf = upto(lift, t); ln_lfw.set_data(lf[:, 0], lf[:, 1]); lp = upto(lj, t); ln_lph.set_data(lp[:, 0], lp[:, 1] * 1000)
    lv = lf[-1]; lift_val.set_text(f"{lv[1]:5.1f} mm  permit {'on' if lv[4] else 'off'}")
    A = upto(arm, t)
    for j in (1, 2, 4): ln_pos[j].set_data(A[:, 0], A[:, j])
    for j in (2, 4): ln_eff[j].set_data(A[:, 0], A[:, 7 + j])
    G = upto(grav, t)
    for j in (2, 4): ln_gff[j].set_data(G[:, 0], G[:, j])
    for b in BUSES: R = upto(rate[b], t); ln_rate[b].set_data(R[:, 0], np.maximum(R[:, 1], 1e-3))
    F = upto(fing, t); ln_f.set_data(F[:, 0], F[:, 1]); I = upto(item, t); ln_iz.set_data(I[:, 0], (I[:, 3] - item_z0) * 10)
    for c in (cl, ca, ct, cr, cg): c.set_xdata([t, t])
    d = at(drv, t); b_ = at(bms, t)
    st_txt.set_text(
        f"lift   CTL-017 controller, host-side emu  vcan_aux   pos {lv[1]:6.1f} mm  {lv[2]:+5.1f} mm/s  state {int(lv[3])}  permit {'on ' if lv[4] else 'off'}  {lv[5]:4.0f} mA\n"
        f"drive  ZLAC8015D emulator                vcan_drive statusword {int(d[1])}  fault {int(d[6])}  target rpm L {d[2]:+4.0f} R {d[3]:+4.0f}\n"
        f"bms    Daly BMS emulator                 vcan_bms   SOC {b_[1]:.1f} %  pack {b_[2]:.1f} V  {b_[3]:+.2f} A\n"
        f"STM32 base controller: host stand-in (stm32: host), not firmware in Renode")
    rows = []
    for b in BUSES:
        st = can[b + "__sim"]; i = np.searchsorted(st, t, side="right")
        for j in range(max(i - (2 if b.startswith("vcan_arm") else 1), 0), i):
            rows.append((st[j], b, int(can[b + "__id"][j]), can[b + "__data"][j].decode()))
    rows.sort()
    can_txt.set_text("\n".join(f"{s:7.3f}  {b:<10} {i:>8X}  {dd[:18]:<18}  {cd.decode(b, i, dd)}"[:118] for s, b, i, dd in rows))


def render(i0, i1, out):
    p = subprocess.Popen(["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS),
                          "-i", "-", "-c:v", "libx264", "-preset", "medium", "-crf", "20", "-pix_fmt", "yuv420p", out],
                         stdin=subprocess.PIPE)
    for i in range(i0, i1):
        draw(SIM0 + i / FPS); fig.canvas.draw()
        p.stdin.write(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].tobytes())
    p.stdin.close(); p.wait()

if __name__ == "__main__":
    n = int(round((SIM1 - SIM0) * FPS))
    if sys.argv[1] == "still":
        for t in map(float, sys.argv[2:]):
            draw(t); fig.savefig(f"still_c_{t:05.1f}.png", dpi=100)
    else:
        part, parts = int(sys.argv[2]), int(sys.argv[3]); a = n * part // parts; b = n * (part + 1) // parts
        render(a, b, f"c_part{part}.mp4"); print("done", a, b)
