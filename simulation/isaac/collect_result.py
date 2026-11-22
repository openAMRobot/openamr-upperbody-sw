#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Build the Isaac run result (isaac_result.json) from the files the run kit wrote, and compare Isaac with MuJoCo and Gazebo.

  python3 simulation/isaac/collect_result.py --kit build/isaac_kit [--mujoco <benchmark_mujoco.json>] [--gazebo <benchmark_gazebo.json>]
      [--stamp <run stamp>]

Reads, per configuration, <kit>/sim/<config>/isaac/: isaac_job.json, isaac_import_log.json, the USD files, check_usd.json; and
<kit>/benchmark_isaac.json, <kit>/preflight.json. Writes <kit>/isaac_result.json and <kit>/isaac_comparison.md.

Status per check: PASS, FAIL, or BLOCKED when the input does not exist (Isaac Sim was not run, which needs a GPU). The overall
status is PASS only when every check passed. Exit 0 PASS, 1 FAIL, 3 BLOCKED. Needs no GPU and no Isaac Sim. Simulation evidence only.

With --stamp (run_on_gpu.sh passes its run stamp, which the Isaac-side scripts write as run_stamp from OPENAMR_RUN_STAMP), an
isaac_import_log.json, check_usd.json or benchmark_isaac.json whose run_stamp differs is the output of another run: BLOCKED, never
read as evidence of this one.
"""
import argparse
import hashlib
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from openamr_sim import benchmark as bm  # noqa: E402

SCHEMA = "openamr.isaac_result.v1"


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def read(path):
    p = pathlib.Path(path)
    return json.loads(p.read_text()) if p.is_file() else None


def moving_joints(job_dir, job):
    """Non-fixed joints of the importer URDF: the DOF count one articulation must have."""
    from openamr_sim.urdf_model import URDF
    u = URDF((job_dir / job["urdf"]).read_text())
    return sorted(j.name for j in u.joints.values() if j.type != "fixed")


def collect(kit, mujoco=None, gazebo=None, stamp=None):
    kit = pathlib.Path(kit)
    checks, configs = [], {}

    def check(name, status, detail):
        checks.append({"check": name, "status": status, "detail": detail})

    def stale(doc):
        """The run_stamp of a doc that another run wrote, or None when the doc belongs to this run (or no stamp is required)."""
        if stamp is None or doc is None or doc.get("run_stamp") == stamp:
            return None
        return f"stale: run_stamp {doc.get('run_stamp')!r}, this run is {stamp!r}"

    for job_path in sorted(kit.glob("sim/*/isaac/isaac_job.json")):
        job_dir = job_path.parent
        job = json.loads(job_path.read_text())
        cfg = job["configuration"]
        log = read(job_dir / "isaac_import_log.json")
        usd_files = sorted(p for p in job_dir.rglob("*") if p.is_file() and p.suffix in (".usd", ".usda", ".usdc"))
        entry = {"expansion_sha256": job["expansion_sha256"], "lock_status": job.get("lock_status"),
                 "usd_sha256": {str(p.relative_to(job_dir)): sha256(p) for p in usd_files}, "import_log": log}
        configs[cfg] = entry
        if log is None:
            check(f"{cfg}.usd_generated", "BLOCKED", "no isaac_import_log.json: isaac_generate_usd.py was not run (needs a GPU)")
            continue
        if stale(log):
            check(f"{cfg}.usd_generated", "BLOCKED", f"isaac_import_log.json {stale(log)}")
            continue
        check(f"{cfg}.usd_generated", "PASS" if log.get("result") == "written" and usd_files else "FAIL", log.get("result"))
        roots = log.get("articulation_roots")
        check(f"{cfg}.one_articulation", "BLOCKED" if roots is None else ("PASS" if len(roots) == 1 else "FAIL"), roots)
        version = log.get("isaac_sim_version") or log.get("isaac_sim_build")   # isaac_sim_build: logs written before 175cd43
        if log.get("version_warning"):
            check(f"{cfg}.isaac_version", "FAIL", log["version_warning"])
        elif version:
            check(f"{cfg}.isaac_version", "PASS", {"isaac_sim_version": version, "source": log.get("isaac_sim_version_source"),
                                                   "kit_build": log.get("kit_build")})
        else:
            check(f"{cfg}.isaac_version", "FAIL", "the import log records no Isaac Sim version")
        report = read(job_dir / "check_usd.json")
        if report is None:
            check(f"{cfg}.check_usd", "BLOCKED", "no check_usd.json")
        elif stale(report):
            check(f"{cfg}.check_usd", "BLOCKED", f"check_usd.json {stale(report)}")
        else:
            s = report["summary"]
            check(f"{cfg}.check_usd", "PASS" if s["failures"] == 0 and s["checks"] > 0 else "FAIL",
                  {"checks": s["checks"], "failures": s["failures"], "first_failures": report["failures"][:10]})
        entry["expected_moving_joints"] = len(moving_joints(job_dir, job))
    bench = read(kit / "benchmark_isaac.json")
    if stale(bench):
        check("benchmark.ran", "BLOCKED", f"benchmark_isaac.json {stale(bench)}")
        bench = None
    elif bench is None:
        check("benchmark.ran", "BLOCKED", "no benchmark_isaac.json: isaac_scenarios.py was not run (needs a GPU)")
    else:
        errors = {k: v for k, v in (bench.get("meta", {}).get("errors") or {}).items() if k != "screenshot"}
        check("benchmark.ran", "FAIL" if errors else "PASS", errors or sorted(bench["results"]))
        cfg = bench.get("configuration")
        expected = configs.get(cfg, {}).get("expected_moving_joints")
        dofs = bench.get("meta", {}).get("dof_count")
        if expected is not None and dofs is not None:
            check(f"{cfg}.dof_count", "PASS" if dofs == expected else "FAIL", {"isaac_dofs": dofs, "urdf_moving_joints": expected})
    docs = {"gazebo": read(gazebo) if gazebo else None, "mujoco": read(mujoco) if mujoco else None, "isaac": bench}
    reference = "gazebo" if docs["gazebo"] else "mujoco"
    rows = bm.compare({k: v for k, v in docs.items() if v is not None or k in ("isaac", reference)}, reference=reference)
    statuses = {c["status"] for c in checks}
    overall = "FAIL" if "FAIL" in statuses else ("BLOCKED" if "BLOCKED" in statuses or not checks else "PASS")
    result = {"schema": SCHEMA, "status": overall, "run_stamp": stamp, "checks": checks, "configurations": configs,
              "preflight": read(kit / "preflight.json"), "comparison_reference": reference, "comparison": rows,
              "note": "Simulation evidence only. Comparison FAIL rows are findings to explain, not part of the status."}
    (kit / "isaac_result.json").write_text(json.dumps(result, indent=1, default=str) + "\n")
    (kit / "isaac_comparison.md").write_text(bm.table(rows, reference=reference) + "\n")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--kit", required=True)
    parser.add_argument("--mujoco")
    parser.add_argument("--gazebo")
    parser.add_argument("--stamp", help="the run stamp every Isaac-side output must carry as run_stamp")
    opts = parser.parse_args()
    result = collect(opts.kit, opts.mujoco, opts.gazebo, opts.stamp)
    for c in result["checks"]:
        print(f"{c['status']:8} {c['check']}")
    print(f"overall: {result['status']}")
    return {"PASS": 0, "FAIL": 1}.get(result["status"], 3)


if __name__ == "__main__":
    sys.exit(main())
