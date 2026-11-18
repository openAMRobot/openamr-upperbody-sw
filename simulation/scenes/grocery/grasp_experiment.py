#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""MuJoCo experiment: grasp and lift of the target product over a sweep of friction and grip settings.

MuJoCo experiment, not ROS or firmware integrated. It informs the gripper-force question of openamr-upperbody-sw pull request 73
(grip force set by the finger contact angle times the position gain, against finger friction 1.0 and product mass 0.15 kg, all
estimated). It changes no shared parameter: each run edits its own in-memory model only (see pick.py).

  python3 simulation/scenes/grocery/grasp_experiment.py --scene build/grocery/mujoco/grocery_scene.xml --out build/grocery/experiment

Writes results.json (every run, summary and decimated time series) and results.md (the table). Exit status 0 when every run
completed; the table reports held or dropped, which is a result, not a pass or fail.
"""
import argparse
import itertools
import json
import pathlib
import platform
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import pick  # noqa: E402

# The sweep. Friction is the product/finger pair coefficient; the grip command is the closed finger target (0 = Build 1 close
# command); the gain factor scales the generated gripper position gain (kp 5.0 N m/rad) for that run only.
FRICTION = (0.2, 0.3, 0.5, 1.0)
GRIP_COMMAND = (0.0, 0.2, 0.25)
KP_FACTOR = (0.25, 0.5, 2.0)          # at friction 0.5 and grip command 0.0
COLUMNS = [("friction", "mu"), ("grip_command", "close cmd [rad]"), ("gripper_kp", "kp [N m/rad]"),
           ("finger_q_at_contact_hold", "finger q on product [rad]"), ("grip_torque_hold", "grip torque [N m]"),
           ("normal_force_per_finger_held_mean", "normal / finger [N]"), ("static_need_per_finger", "need m g/(2 mu) [N]"),
           ("margin", "margin"), ("lift_travel", "lift travel [m]"), ("grasp_rise", "hand rise [m]"),
           ("product_rise", "product rise [m]"), ("slip_z_max", "slip max [m]"), ("slip_rate_hold", "creep in hold [m/s]"),
           ("held", "held")]


def sweep():
    runs = [pick.Settings(friction=mu, grip_command=cmd) for mu, cmd in itertools.product(FRICTION, GRIP_COMMAND)]
    runs += [pick.Settings(friction=0.5, grip_command=0.0, kp_factor=k) for k in KP_FACTOR]
    return runs


def run_one(scene, settings):
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(scene))
    d = mujoco.MjData(m)
    return pick.Pick(m, d, settings=settings).run()


def _fmt(v):
    if isinstance(v, bool):
        return "yes" if v else "NO"
    if isinstance(v, float):
        return f"{v:.4g}"
    return str(v)


def table(results):
    head = "| " + " | ".join(c[1] for c in COLUMNS) + " |"
    rule = "|" + "|".join("---" for _ in COLUMNS) + "|"
    rows = []
    for r in results:
        values = []
        for key, _ in COLUMNS:
            values.append(_fmt(r["settings"][key] if key in r["settings"] else r[key]))
        rows.append("| " + " | ".join(values) + " |")
    return "\n".join([head, rule, *rows])


def main():
    import mujoco
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scene", required=True)
    parser.add_argument("--out", default="build/grocery/experiment")
    parser.add_argument("--quick", action="store_true", help="the baseline run only (friction 1.0, close command 0)")
    opts = parser.parse_args()
    out = pathlib.Path(opts.out)
    out.mkdir(parents=True, exist_ok=True)
    runs = [pick.Settings()] if opts.quick else sweep()
    results = []
    for s in runs:
        r = run_one(opts.scene, s)
        series = r.pop("series")
        r["series_decimated"] = {k: v[::4] for k, v in series.items() if isinstance(v, list)}
        results.append(r)
        print(json.dumps({k: r[k] for k in ("settings", "normal_force_per_finger_held_mean", "product_rise", "slip_z_max", "held")}))
    meta = {"label": "MuJoCo experiment, not ROS or firmware integrated", "mujoco": mujoco.__version__,
            "python": platform.python_version(), "scene": str(opts.scene),
            "controller": "pick.py: position actuators of the generated model, gravity feed-forward on the arm joints, lift ramp 18 mm/s",
            "timestep": None}
    m = mujoco.MjModel.from_xml_path(str(opts.scene))
    meta["timestep"] = m.opt.timestep
    meta["cone"] = "elliptic" if m.opt.cone == mujoco.mjtCone.mjCONE_ELLIPTIC else "pyramidal"
    meta["impratio"] = m.opt.impratio
    (out / "results.json").write_text(json.dumps({"meta": meta, "runs": results}, indent=1, default=float) + "\n")
    md = ["# Grasp and lift sweep (MuJoCo experiment, not ROS or firmware integrated)", "",
          f"MuJoCo {meta['mujoco']}, timestep {meta['timestep']:.6g} s, {meta['cone']} cone, impratio {meta['impratio']:.3g}. "
          "Product 0.15 kg, 0.035 x 0.035 x 0.14 m (Build 1 values). Normal force: mean per finger while lifted and held. "
          "Hand rise below lift travel is arm sag under the payload (the feed-forward has no payload term).", "", table(results), ""]
    (out / "results.md").write_text("\n".join(md))
    print((out / "results.md").read_text())
    return 0


if __name__ == "__main__":
    sys.exit(main())
