# SPDX-License-Identifier: MIT
"""Run the backend benchmark scenarios in Isaac Sim on the generated USD. Runs INSIDE Isaac Sim (its python.sh), on a GPU machine.

Same scenarios, command shaping and metrics as the Gazebo and MuJoCo runners (openamr_sim/benchmark.py, scenarios.yaml): the USD's
own physics scene (time steps from the job), the job's drives, commands written every controller period (750 Hz, every second
physics step at 1500 Hz). Ground truth is the PhysX body poses. Nothing is tuned for Isaac.

  /isaac-sim/python.sh simulation/isaac/isaac_scenarios.py --job <kit>/sim/bp000/isaac/isaac_job.json \
      --spec <kit>/scenarios.json --out <kit>/benchmark_isaac.json [--screenshot <kit>/isaac_rest.png]

Writes the result document (openamr.sim_benchmark.v1, backend isaac) even when a scenario fails: a failed scenario has no values and
reads as BLOCKED in the comparison. Simulation evidence only: not hardware, physical acceptance or safety evidence.
"""
import argparse
import json
import os
import math
import pathlib
import sys
import time
import traceback

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from openamr_sim import benchmark as bm  # noqa: E402

LEFT = [f"openarm_left_joint{k}" for k in range(1, 8)]


def quat_to_R(q):  # q = (w, x, y, z)
    w, x, y, z = (float(v) for v in q)
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--job", required=True)
    parser.add_argument("--spec", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--screenshot")
    opts = parser.parse_args()
    job_path = pathlib.Path(opts.job).resolve()
    job = json.loads(job_path.read_text())
    spec = bm.load_scenarios(opts.spec)
    usd = job_path.parent / job["usd"]
    doc = {"schema": bm.SCHEMA, "backend": "isaac", "configuration": job["configuration"], "start_keyframe": spec["start_keyframe"],
           "expansion_sha256": job["expansion_sha256"], "results": {}, "meta": {"errors": {}},
           "run_stamp": os.environ.get("OPENAMR_RUN_STAMP")}  # the run kit's stamp; collect_result.py --stamp requires it

    from isaacsim import SimulationApp  # noqa: E402  (only inside Isaac Sim)
    app = SimulationApp({"headless": True})
    started = time.monotonic()
    try:
        import numpy as np
        import omni.kit.app
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.api.objects import DynamicCuboid
        from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
        from isaacsim.core.utils.types import ArticulationAction
        from pxr import UsdPhysics

        doc["meta"]["isaac_sim_build"] = omni.kit.app.get_app().get_build_version()
        omni.usd.get_context().open_stage(str(usd))
        for _ in range(10):
            app.update()
        stage = omni.usd.get_context().get_stage()
        scenes = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
        roots = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
        doc["meta"]["physics_scenes"] = [str(p.GetPath()) for p in scenes]
        doc["meta"]["articulation_roots"] = [str(p.GetPath()) for p in roots]
        if len(scenes) != 1 or len(roots) != 1:
            raise RuntimeError(f"expected one physics scene and one articulation, found {len(scenes)} and {len(roots)}")
        dt = 1.0 / float(job["physics"]["time_steps_per_second"])
        world = World(physics_dt=dt, rendering_dt=dt, stage_units_in_meters=1.0, physics_prim_path=str(scenes[0].GetPath()))
        world.scene.add_default_ground_plane()

        def body_path(name):
            hits = [p for p in stage.Traverse() if p.GetName() == name and p.HasAPI(UsdPhysics.RigidBodyAPI)]
            if len(hits) != 1:
                raise RuntimeError(f"{len(hits)} rigid bodies named {name}")
            return str(hits[0].GetPath())

        robot = world.scene.add(SingleArticulation(prim_path=str(roots[0].GetPath()), name="openamrobot"))
        base = SingleRigidPrim(body_path("base_footprint"), name="base_footprint")
        ee = SingleRigidPrim(body_path(spec["scenarios"]["arm_reach"]["link"]), name="ee")
        box_spec = spec["scenarios"]["object_contact"]
        box = world.scene.add(DynamicCuboid(prim_path="/World/bench_box", name="bench_box", position=np.array(box_spec["position"]),
                                            size=box_spec["size"], mass=box_spec["mass"]))
        world.reset()
        names = list(robot.dof_names)
        doc["meta"]["dof_count"] = len(names)
        doc["meta"]["dof_names"] = names
        index = {n: i for i, n in enumerate(names)}
        period = int(round((1.0 / 750.0) / dt)) or 1
        doc["meta"]["controller_period_steps"] = period
        state = {"k": 0, "box_free": False}
        release = np.array(box_spec["position"], dtype=float)

        def q(name):
            return float(robot.get_joint_positions()[index[name]])

        def base_pose():
            p, quat = base.get_world_pose()
            return quat_to_R(quat), [float(v) for v in p]

        def hold_box():
            box.set_world_pose(position=release, orientation=np.array([1.0, 0.0, 0.0, 0.0]))
            box.set_linear_velocity(np.zeros(3))
            box.set_angular_velocity(np.zeros(3))

        def step(seconds, on_ctrl=None, on_step=None):
            n = int(round(seconds / dt))
            for i in range(n):
                if on_ctrl is not None and state["k"] % period == 0:
                    on_ctrl(i * dt)
                if not state["box_free"]:
                    hold_box()
                world.step(render=False)
                state["k"] += 1
                if on_step is not None:
                    on_step(i * dt)

        def targets(pos=None, vel=None):
            if pos:
                robot.apply_action(ArticulationAction(joint_positions=np.array(list(pos.values()), dtype=float),
                                                      joint_indices=np.array([index[n] for n in pos])))
            if vel:
                robot.apply_action(ArticulationAction(joint_velocities=np.array(list(vel.values()), dtype=float),
                                                      joint_indices=np.array([index[n] for n in vel])))

        # start state: every position drive targets its current position (L1000_home is the zero pose)
        targets({n: q(n) for n in names if n in job["drives"] and job["drives"][n]["mode"] == "position"})
        sc = spec["scenarios"]

        def scenario(name, fn):
            try:
                doc["results"][name] = {k: float(v) for k, v in fn().items()}
            except Exception as err:  # noqa: BLE001  (recorded; the comparison shows BLOCKED)
                doc["meta"]["errors"][name] = f"{type(err).__name__}: {err}"
                traceback.print_exc()

        def rest():
            step(spec["settle_s"])
            R, p = base_pose()
            pitch, roll = bm.pitch_roll_of(R)
            if opts.screenshot:
                try:
                    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
                    from isaacsim.core.utils.viewports import set_camera_view
                    set_camera_view(eye=[2.5, -2.5, 1.8], target=[0.0, 0.0, 0.8])
                    for _ in range(30):
                        world.step(render=True)
                    capture_viewport_to_file(get_active_viewport(), opts.screenshot)
                    for _ in range(30):
                        app.update()
                    doc["meta"]["screenshot"] = opts.screenshot
                except Exception as err:  # noqa: BLE001
                    doc["meta"]["errors"]["screenshot"] = f"{type(err).__name__}: {err}"
            return {"base_height_m": p[2], "base_pitch_deg": pitch, "base_roll_deg": roll}

        def joint_step():
            s = sc["joint_step"]
            jn, q0 = s["joint"], q(s["joint"])
            _, p0 = base_pose()
            step(s["move_s"], lambda t: targets({jn: bm.cubic(q0, s["target"], s["move_s"], t)}))
            hold = []
            step(s["hold_s"], None, lambda t: hold.append(abs(q(jn) - s["target"])))
            _, p1 = base_pose()
            return {"final_error_rad": abs(q(jn) - s["target"]), "max_hold_error_rad": max(hold),
                    "base_displacement_m": math.hypot(p1[0] - p0[0], p1[1] - p0[1])}

        def arm_reach():
            s = sc["arm_reach"]
            start = {j: q(j) for j in s["targets"]}
            step(s["move_s"], lambda t: targets({j: bm.cubic(start[j], v, s["move_s"], t) for j, v in s["targets"].items()}))
            step(s["hold_s"])
            Rb, pb = base_pose()
            pe, _ = ee.get_world_pose()
            e = bm.in_base(Rb, pb, [float(v) for v in pe])
            return {"ee_x_m": e[0], "ee_y_m": e[1], "ee_z_m": e[2],
                    "max_joint_error_rad": max(abs(q(j) - v) for j, v in s["targets"].items())}

        def lift_move():
            s = sc["lift_move"]
            vel = float(job["lift_velocity_limit"])
            ramp_s = abs(s["to"] - s["from"]) / vel
            R0, _ = base_pose()
            track = {"max": 0.0, "target": s["from"]}

            def ctrl(t):
                track["target"] = bm.lift_ramp(s["from"], s["to"], vel, t)
                targets({"lift_joint": track["target"]})
            step(ramp_s + s["hold_s"], ctrl, lambda t: track.__setitem__("max", max(track["max"], abs(track["target"] - q("lift_joint")))))
            R1, _ = base_pose()
            return {"final_position_m": q("lift_joint"), "max_tracking_error_m": track["max"],
                    "base_pitch_change_deg": bm.pitch_roll_of(R1)[0] - bm.pitch_roll_of(R0)[0], "ramp_velocity_m_s": vel}

        dd = job.get("diffdrive")

        def drive(name, lin, ang, on_key):
            s = sc[name]
            if dd is None:
                raise RuntimeError("the job carries no DiffDrive values (generate the job with this kit)")
            ctrl_dt = period * dt
            lp, _ = bm.shaped_profile(lin, s[on_key], s["stop_s"], dd["max_linear_velocity"], dd["max_linear_acceleration"],
                                      dd["min_linear_acceleration"], ctrl_dt)
            ap, ang_int = bm.shaped_profile(ang, s[on_key], s["stop_s"], dd["max_angular_velocity"], dd["max_angular_acceleration"],
                                            dd["min_angular_acceleration"], ctrl_dt)
            R0, p0 = base_pose()
            yaws = [bm.yaw_of(R0)]

            def ctrl(t):
                i = min(int(round(t / ctrl_dt)), len(lp) - 1)
                left, right = bm.wheel_speeds(lp[i], ap[i], dd["wheel_separation"], dd["wheel_radius"])
                targets(vel={"left_wheel_joint": left, "right_wheel_joint": right})
                yaws.append(bm.yaw_of(base_pose()[0]))
            step(s[on_key] + s["stop_s"], ctrl)
            targets(vel={"left_wheel_joint": 0.0, "right_wheel_joint": 0.0})
            R1, p1 = base_pose()
            yaws.append(bm.yaw_of(R1))
            mo = bm.planar_motion(p0, yaws[0], p1, yaws)
            if name == "straight_drive":
                return {"distance_m": mo["along_m"], "lateral_drift_m": mo["lateral_m"], "yaw_change_rad": mo["yaw_change_rad"]}
            return {"yaw_over_commanded": mo["yaw_change_rad"] / ang_int, "centre_drift_m": mo["displacement_m"],
                    "commanded_yaw_rad": ang_int}

        def object_contact():
            s = sc["object_contact"]
            hold_box()
            state["box_free"] = True
            step(s["settle_s"])
            p, _ = box.get_world_pose()
            v = box.get_linear_velocity()
            return {"rest_height_m": float(p[2]), "horizontal_slip_m": math.hypot(float(p[0]) - release[0], float(p[1]) - release[1]),
                    "final_speed_m_s": float(np.linalg.norm(v))}

        scenario("rest", rest)
        scenario("joint_step", joint_step)
        scenario("arm_reach", arm_reach)
        scenario("lift_move", lift_move)
        scenario("straight_drive", lambda: drive("straight_drive", sc["straight_drive"]["linear"], 0.0, "drive_s"))
        scenario("in_place_turn", lambda: drive("in_place_turn", 0.0, sc["in_place_turn"]["angular"], "turn_s"))
        scenario("object_contact", object_contact)
    except Exception as err:  # noqa: BLE001
        doc["meta"]["errors"]["setup"] = f"{type(err).__name__}: {err}"
        traceback.print_exc()
    finally:
        doc["meta"]["wall_s"] = round(time.monotonic() - started, 1)
        pathlib.Path(opts.out).write_text(json.dumps(doc, indent=1, default=str) + "\n")
        app.close()
    return 0 if not doc["meta"]["errors"] or set(doc["meta"]["errors"]) <= {"screenshot"} else 1


if __name__ == "__main__":
    sys.exit(main())
