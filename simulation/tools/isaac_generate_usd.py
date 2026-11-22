# SPDX-License-Identifier: MIT
"""Generate the Isaac Sim USD of the combined model from an import job. Runs INSIDE Isaac Sim (its python.sh), on a GPU machine.

The job comes from simulation/tools/generate.py --isaac (no GPU needed): the URDF of the pinned expansion with absolute mesh paths,
and isaac_job.json with the importer settings and drive values of engine_overrides.yaml.

  # on a GPU machine, from the repository root, with the job in build/sim/bp000/isaac (see docs/simulators.md)
  docker run --rm --gpus all -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -v "$PWD":/work -w /work --entrypoint /isaac-sim/python.sh \
      nvcr.io/nvidia/isaac-sim:5.1.0 simulation/tools/isaac_generate_usd.py --job build/sim/bp000/isaac/isaac_job.json

Writes the USD next to the job and isaac_import_log.json. The USD is a build product: never commit it and never edit it.
Then run simulation/tools/check_usd.py on it (no GPU needed).
"""
import argparse
import json
import math
import pathlib
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--job", required=True)
    opts = parser.parse_args()
    job_path = pathlib.Path(opts.job).resolve()
    job = json.loads(job_path.read_text())
    out = job_path.parent

    from isaacsim import SimulationApp  # noqa: E402  (only inside Isaac Sim)

    app = SimulationApp({"headless": True})
    log = {"job": str(job_path), "expansion_sha256": job["expansion_sha256"], "settings": {}, "missing_settings": []}
    try:
        import omni.kit.app
        import omni.kit.commands
        from pxr import PhysxSchema, Sdf, Usd, UsdPhysics

        version = omni.kit.app.get_app().get_build_version()
        log["isaac_sim_build"] = version
        if not version.startswith(job["isaac_sim_version"]):
            log["version_warning"] = f"job pins {job['isaac_sim_version']}, running {version}"
        from isaacsim.asset.importer.urdf import _urdf

        status, config = omni.kit.commands.execute("URDFCreateImportConfig")
        for key, value in job["importer"].items():
            if key in ("note", "default_drive_type"):
                continue
            if hasattr(config, key):
                setattr(config, key, value)
                log["settings"][key] = value
            else:
                log["missing_settings"].append(key)
        drive_types = {"position": _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION,
                       "velocity": _urdf.UrdfJointTargetType.JOINT_DRIVE_VELOCITY}
        config.default_drive_type = drive_types[job["importer"]["default_drive_type"]]
        required = {"merge_fixed_joints", "fix_base", "import_inertia_tensor", "self_collision", "parse_mimic"}
        if required & set(log["missing_settings"]):
            raise RuntimeError(f"importer has no setting {sorted(required & set(log['missing_settings']))}; this Isaac Sim is not the pinned one")
        usd_path = out / job["usd"]
        status, prim_path = omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=str(out / job["urdf"]),
                                                      import_config=config, dest_path=str(usd_path), get_articulation_root=True)
        if not status:
            raise RuntimeError("URDFParseAndImportFile failed")
        log["articulation_root"] = str(prim_path)
        # Drives, armature and the physics scene: written into the root layer of the imported USD, from the job only.
        stage = Usd.Stage.Open(str(usd_path))
        joints = {p.GetName(): p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)}
        for name, spec in job["drives"].items():
            prim = joints[name]
            drive = UsdPhysics.DriveAPI.Apply(prim, spec["type"])
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(spec["stiffness"])
            drive.CreateDampingAttr(spec["damping"])
            if spec["max_force"] is not None:
                drive.CreateMaxForceAttr(spec["max_force"])
        for name, value in job["armature"].items():
            api = PhysxSchema.PhysxJointAPI.Apply(joints[name])
            api.CreateArmatureAttr(value)
        scenes = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
        if len(scenes) != 1:
            raise RuntimeError(f"expected one physics scene, found {len(scenes)}")
        scene = PhysxSchema.PhysxSceneAPI.Apply(scenes[0])
        scene.CreateTimeStepsPerSecondAttr(job["physics"]["time_steps_per_second"])
        g = job["physics"]["gravity"]
        UsdPhysics.Scene(scenes[0]).CreateGravityMagnitudeAttr(math.sqrt(sum(x * x for x in g)))
        stage.GetRootLayer().customLayerData = {"openamrobot_expansion_sha256": job["expansion_sha256"],
                                               "generated_by": "simulation/tools/isaac_generate_usd.py"}
        stage.Save()
        log["usd"] = str(usd_path)
        log["result"] = "written"
    except Exception as err:  # noqa: BLE001  (the log records any failure)
        log["result"] = f"FAIL: {err}"
    finally:
        (out / "isaac_import_log.json").write_text(json.dumps(log, indent=1) + "\n")
        app.close()
    return 0 if log.get("result") == "written" else 1


if __name__ == "__main__":
    sys.exit(main())
