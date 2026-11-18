# SPDX-License-Identifier: MIT
"""Render the Isaac Sim grocery stage on a GPU machine. Runs INSIDE Isaac Sim 5.1 (its python.sh). Never run yet: no GPU was
available to the author or to the hosted runners, so this step is BLOCKED and nothing it would produce exists.

Written against the Isaac Sim 5.1.0 documentation (RTX Lidar Sensor page, IsaacSensorCreateRtxLidar; Replicator render products and
the rgb annotator). Every call is logged; an API that differs in the running build is recorded as a failure, never skipped.

  # 1. generator output and robot USD (docs/simulators.md, section Isaac Sim), on the GPU machine
  # 2. the stage, with usd-core (no GPU):
  python3 simulation/scenes/grocery/compose_isaac.py --robot-usd build/sim/bp000/isaac/openamrobot_bp000.usd \
      --job build/sim/bp000/isaac/isaac_job.json --out build/grocery/isaac/grocery_store.usda
  # 3. the render:
  docker run --rm --gpus all -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -v "$PWD":/work -w /work --entrypoint /isaac-sim/python.sh \
      nvcr.io/nvidia/isaac-sim:5.1.0 simulation/scenes/grocery/isaac_render.py --stage build/grocery/isaac/grocery_store.usda \
      --out build/grocery/isaac/render

Writes one PNG per robot camera and the overview camera after `--settle` seconds of physics, the RTX LiDAR return count, and
render_log.json. The robot only holds its drive targets: no ROS bridge, no controller, no firmware. Not ROS or firmware integrated.
"""
import argparse
import json
import pathlib
import sys

LIDAR_CONFIG = "Example_Rotary"   # no RPLIDAR S3 profile was found in the 5.1 library; the placement is the authoritative frame


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--settle", type=float, default=2.0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    opts = parser.parse_args()
    out = pathlib.Path(opts.out)
    out.mkdir(parents=True, exist_ok=True)

    from isaacsim import SimulationApp  # noqa: E402  (only inside Isaac Sim)

    app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})
    log = {"stage": opts.stage, "steps": [], "result": "not started"}
    try:
        import numpy as np
        import omni.kit.app
        import omni.kit.commands
        import omni.replicator.core as rep
        import omni.usd
        from isaacsim.core.api import World
        from PIL import Image
        from pxr import Gf, UsdGeom

        log["isaac_sim_build"] = omni.kit.app.get_app().get_build_version()
        omni.usd.get_context().open_stage(str(pathlib.Path(opts.stage).resolve()))
        stage = omni.usd.get_context().get_stage()
        world = World(stage_units_in_meters=1.0)
        cameras = [p for p in stage.Traverse() if p.IsA(UsdGeom.Camera) and p.GetName().endswith("_rtx")]
        mounts = [p for p in stage.Traverse() if p.GetName() == "rtx_lidar_mount"]
        log["steps"].append({"cameras": [str(p.GetPath()) for p in cameras], "lidar_mounts": [str(p.GetPath()) for p in mounts]})
        if len(cameras) != 4 or len(mounts) != 1:
            raise RuntimeError("the stage lacks the sensor prims of compose_isaac.py (was the robot USD resolved?)")
        lidar_path = mounts[0].GetPath().AppendChild("rtx_lidar")
        ok, lidar = omni.kit.commands.execute("IsaacSensorCreateRtxLidar", path=str(lidar_path), parent=None,
                                              config=LIDAR_CONFIG, translation=Gf.Vec3d(0, 0, 0),
                                              orientation=Gf.Quatd(1, 0, 0, 0))
        log["steps"].append({"lidar_created": bool(ok), "lidar": str(lidar_path), "config": LIDAR_CONFIG})
        overview = UsdGeom.Camera.Define(stage, "/World/overview_camera")
        UsdGeom.XformCommonAPI(overview).SetTranslate(Gf.Vec3d(-2.6, -3.4, 3.0))
        UsdGeom.XformCommonAPI(overview).SetRotate(Gf.Vec3f(58.0, 0.0, -38.0))
        products = {}
        for prim in cameras + [overview.GetPrim()]:
            rp = rep.create.render_product(str(prim.GetPath()), (opts.width, opts.height))
            annot = rep.AnnotatorRegistry.get_annotator("rgb")
            annot.attach([rp])
            products[prim.GetName()] = annot
        lidar_annot = None
        try:
            lrp = rep.create.render_product(str(lidar_path), [1, 1], name="rtx_lidar")
            lidar_annot = rep.AnnotatorRegistry.get_annotator("IsaacExtractRTXSensorPointCloudNoAccumulator")
            lidar_annot.attach([lrp])
        except Exception as err:  # noqa: BLE001  (recorded, the camera renders still count)
            log["steps"].append({"lidar_annotator": f"FAIL: {err}"})
        world.reset()
        steps = int(opts.settle * 60)
        for _ in range(steps):
            world.step(render=True)
        rep.orchestrator.step()
        for name, annot in products.items():
            rgba = annot.get_data()
            Image.fromarray(np.asarray(rgba)[:, :, :3]).save(out / f"{name}.png")
            log["steps"].append({"image": f"{name}.png", "shape": list(np.asarray(rgba).shape)})
        if lidar_annot is not None:
            pc = lidar_annot.get_data()
            n = int(len(pc["data"])) if isinstance(pc, dict) and "data" in pc else -1
            log["steps"].append({"lidar_returns": n})
        log["result"] = "rendered"
    except Exception as err:  # noqa: BLE001  (the log records any failure)
        log["result"] = f"FAIL: {err}"
    finally:
        (out / "render_log.json").write_text(json.dumps(log, indent=1) + "\n")
        app.close()
    return 0 if log["result"] == "rendered" else 1


if __name__ == "__main__":
    sys.exit(main())
