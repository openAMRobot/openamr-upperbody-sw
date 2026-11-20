#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Recompute every expansion hash of the lock mirror with the xacro on PATH and compare with the locked values.

Covers all combined-model entries (each at its locked commit and platform-sw pin) and the base-model entries of
simulation/expansion_lock.yaml. Run it when the xacro pin changes: an xacro that changes any expansion fails here before it is
locked. Sources are taken with `git archive <sha>` from local clones; the working trees are not touched. Hash definition and
prefix as openamr_sim.expand (the same as the audits tool).

  source /opt/ros/jazzy/setup.bash
  python3 simulation/tools/reproduce_lock.py --upperbody <clone> --platform-sw <clone> \
      --upstream <dir holding openarm_description and openarm_ros2 at the pins> [--lock FILE] [--json OUT]

Exit 0 when every entry is identical (sha256, bytes, links, joints), 1 otherwise, 3 when xacro is missing.
"""
import argparse
import hashlib
import json
import os
import pathlib
import shutil
import subprocess
import sys
import tempfile

import yaml

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from openamr_sim.expand import LOCK_MIRROR, _make_prefix, _normalise, xacro_identity  # noqa: E402

COMBINED = "ros2/src/openamrobot_upperbody_description/urdf/combined_robot.xacro"
PINS = "ros2/src/openamrobot_upperbody_description/config/upstream_pins.yaml"
BASE = "ros2/src/openamrobot_description/urdf/robo_urdf.urdf.xacro"


def archive(clone, sha, dest):
    dest.mkdir(parents=True)
    tar = subprocess.run(["git", "-C", str(clone), "archive", sha], check=True, capture_output=True).stdout
    subprocess.run(["tar", "-x", "-C", str(dest)], input=tar, check=True)


def expand(prefix, xacro_file, args):
    env = dict(os.environ)
    env["AMENT_PREFIX_PATH"] = f"{prefix}:{env.get('AMENT_PREFIX_PATH', '')}"
    run = subprocess.run(["xacro", str(xacro_file), *[f"{k}:={v}" for k, v in args.items()]], capture_output=True, text=True, env=env)
    if run.returncode != 0:
        raise RuntimeError(f"xacro failed for {xacro_file} {args}: {run.stderr.strip()[-2000:]}")
    return _normalise(run.stdout, prefix)


def compare(text, entry):
    data = text.encode()
    got = {"sha256": hashlib.sha256(data).hexdigest(), "bytes": len(data), "links": text.count("<link "), "joints": text.count("<joint ")}
    want = {k: entry.get(k) for k in got if entry.get(k) is not None}
    return got, all(got[k] == v for k, v in want.items()) and "sha256" in want


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--upperbody", required=True)
    p.add_argument("--platform-sw", required=True)
    p.add_argument("--upstream", required=True)
    p.add_argument("--lock", default=str(LOCK_MIRROR))
    p.add_argument("--json")
    o = p.parse_args(argv)
    if shutil.which("xacro") is None:
        print("BLOCKED: xacro not on PATH", file=sys.stderr)
        return 3
    lock = yaml.safe_load(pathlib.Path(o.lock).read_text())
    upstream = pathlib.Path(o.upstream).resolve()
    rows = []
    with tempfile.TemporaryDirectory() as tmp:
        tmp = pathlib.Path(tmp)
        trees, prefixes = {}, {}

        def tree(clone, sha, tag):
            if sha not in trees:
                trees[sha] = tmp / f"{tag}-{sha[:12]}"
                archive(clone, sha, trees[sha])
            return trees[sha]

        for e in lock["entries"]:
            sha, inp = e["commit"], e["inputs"]
            ub = tree(o.upperbody, sha, "ub")
            pin = yaml.safe_load((ub / PINS).read_text())["pins"]
            if sha not in prefixes:
                prefixes[sha] = tmp / f"prefix-{sha[:12]}"
                _make_prefix(prefixes[sha], [ub, tree(o.platform_sw, pin["openamr_platform_sw"], "pf"),
                                             upstream / "openarm_description", upstream / "openarm_ros2"])
            text = expand(prefixes[sha], ub / COMBINED, {"base_plate": inp["base_plate"], "hardware_profile": inp["hardware_profile"]})
            got, ok = compare(text, e)
            rows.append({"group": "combined_model", "commit": sha, "base_plate": inp["base_plate"], "profile": inp["hardware_profile"],
                         "locked": e["sha256"], **got, "identical": ok})
        for e in lock.get("base_model", []):
            pf = tree(o.platform_sw, e["commit"], "pf")
            prefix = tmp / f"prefix-base-{e['commit'][:12]}"
            _make_prefix(prefix, [pf])
            got, ok = compare(expand(prefix, pf / BASE, {}), e)
            rows.append({"group": "base_model", "commit": e["commit"], "locked": e["sha256"], **got, "identical": ok})
    same = sum(r["identical"] for r in rows)
    for r in rows:
        print(f"{r['group']:14} {r['commit'][:7]} {r.get('base_plate', '-'):5} {r.get('profile', '-'):6} locked {r['locked'][:12]} "
              f"computed {r['sha256'][:12]} {'identical' if r['identical'] else 'DIFFERENT'}")
    identity = xacro_identity()
    print(f"xacro {identity['version']} sources {identity['sources_sha256']}: {same} of {len(rows)} lock entries identical")
    if o.json:
        pathlib.Path(o.json).write_text(json.dumps({"xacro": identity, "identical": same, "total": len(rows), "rows": rows}, indent=1))
    return 0 if rows and same == len(rows) else 1


if __name__ == "__main__":
    sys.exit(main())
