#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Generate the MuJoCo model of a named configuration from the pinned expansion.

  source /opt/ros/jazzy/setup.bash
  python3 simulation/tools/generate.py --config bp000 --platform-sw <checkout at the pin> --openarm-description <checkout at the pin> \
      --out build/sim

Writes <out>/<config>/openamrobot_<config>.xml with its meshes and report_<config>.json. With --isaac it also writes the Isaac Sim
import job in <out>/<config>/isaac/ (the URDF for the importer and isaac_job.json); the USD itself needs Isaac Sim on a GPU machine. The expansion hash is checked against
simulation/expansion_lock.yaml first: a mismatch exits 2, inputs without a lock entry exit 3 (BLOCKED, not a pass).
The output is a build product. Never commit it and never edit it.
"""
import argparse
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from openamr_sim import expand as ex  # noqa: E402
from openamr_sim import isaac  # noqa: E402
from openamr_sim import mjcf  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", action="append", choices=sorted(ex.CONFIGURATIONS), help="repeat for several; default all")
    parser.add_argument("--platform-sw", required=True)
    parser.add_argument("--openarm-description", required=True)
    parser.add_argument("--out", default="build/sim")
    parser.add_argument("--isaac", action="store_true", help="also write the Isaac Sim import job")
    opts = parser.parse_args()
    for name in opts.config or sorted(ex.CONFIGURATIONS):
        try:
            expansion = ex.expand(name, opts.platform_sw, opts.openarm_description)
        except ex.LockMismatch as err:
            print(f"FAIL: {err}", file=sys.stderr)
            return 2
        except ex.NotLocked as err:
            print(f"BLOCKED: {err}", file=sys.stderr)
            return 3
        report = mjcf.generate(expansion, pathlib.Path(opts.out) / name)
        if opts.isaac:
            job = isaac.build_job(expansion, pathlib.Path(opts.out) / name / "isaac")
            report["isaac_job"] = {"drives": len(job["drives"]), "armature": len(job["armature"]), "mimic": sorted(job["mimic"])}
        print(json.dumps({"configuration": name, "expansion_sha256": report["expansion_sha256"], "lock": report["lock_status"],
                          "counts": report["counts"], "unsupported": len(report["unsupported"]),
                          **({"isaac_job": report["isaac_job"]} if "isaac_job" in report else {})}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
