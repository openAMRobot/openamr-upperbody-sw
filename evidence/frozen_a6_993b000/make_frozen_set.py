#!/usr/bin/env python3
"""Produce the frozen expansion set: every configuration of the A6 contract matrix, expanded, with its configuration id.

Needs a ROS 2 Jazzy environment and the workspace's packages on AMENT_PREFIX_PATH. Which description package tree it
expands is decided by where `$(find openamrobot_upperbody_description)` resolves, so to freeze a commit that is not the
installed one, put a prefix holding that commit's package FIRST on AMENT_PREFIX_PATH (see INDEX.md, "Reproduce").

    make_frozen_set.py --scripts-dir <that tree>/scripts --out DIR --commit SHA --branch NAME [--only LABEL ...]

`--scripts-dir` is the `scripts/` of the tree being frozen: its own evidence_common.py and print_config_id.py compute the
id, so the id is the one that tree's own tooling gives. Each file is the xacro output with only its banner (which names a
local path) replaced by a header comment; the id hashes the canonical form with comments dropped, so that does not change
it. verify_config_ids.py proves it from the files alone.
"""
import argparse
import json
import os
import re
import sys
import xml.etree.ElementTree as ET

BANNER_RE = re.compile(r"(<\?xml[^>]*\?>)(<!-- =+ -->)(<!-- \|[^\n]*? \| -->)(<!-- \|[^\n]*? \| -->)(<!-- =+ -->)")
HEADER = ("<!-- FROZEN EXPANSION. FIXED-MAST configuration, superseded by option B (P-03 rev18.9 item 18): this is what "
          "A6 froze, not the robot as it will be built. | label {label} | config_id {config_id} | openamr-upperbody-sw "
          "{commit} ({branch}) | datums revision {datums_revision} | openarm_description {openarm_description} | "
          "base: openamr-platform-sw {platform_sw} (included unchanged, not part of the id) | the id hashes the "
          "canonical form of this document with comments dropped, see verify_config_ids.py and ../INDEX.md -->")


def summary(urdf_text):
    root = ET.fromstring(urdf_text)
    masses = [float(m.get("value")) for m in root.iterfind("./link/inertial/mass")]
    plugins = sorted({p.get("filename") or p.text or "" for p in root.iterfind("./ros2_control/hardware/plugin")})
    return {"links": len(root.findall("link")), "joints": len(root.findall("joint")), "massed_links": len(masses),
            "modelled_mass_kg": round(sum(masses), 3), "ros2_control_hardware_plugins": plugins,
            "has_gazebo_model_plugin": any(p.get("filename") == "gz_ros2_control-system" for p in root.iterfind("./gazebo/plugin"))}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--scripts-dir", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--commit", required=True, help="full commit hash of the tree being frozen")
    ap.add_argument("--branch", required=True)
    ap.add_argument("--platform-sw", required=True, help="full commit hash of the openamr-platform-sw checkout the base came from")
    ap.add_argument("--only", nargs="*", help="labels (mast_id/arm_package/hardware_profile) to produce; default: the whole matrix")
    ns = ap.parse_args(argv)

    sys.path.insert(0, os.path.abspath(ns.scripts_dir))
    import evidence_common as ev
    import print_config_id as pc
    import yaml

    share = ev.share_dir()
    workspace = ev.workspace_root(share)
    commit, source = ev.upstream_commit_and_source(workspace)
    with open(os.path.join(share, "config", "mast_ids.yaml")) as f:
        mast_ids = yaml.safe_load(f)
    revision = pc.datums_revision(os.path.join(share, "config", "datums.yaml"))
    os.makedirs(ns.out, exist_ok=True)

    entries = []
    for mast_id, arm_package, hardware_profile in ev.expansion_matrix(mast_ids):
        label = ev.expansion_label(mast_id, arm_package, hardware_profile)
        if ns.only and label not in ns.only:
            continue
        urdf = ev.expand_urdf(share, mast_id, arm_package, hardware_profile, "false", None)
        config_id = pc.config_id(urdf, mast_id, arm_package, hardware_profile, revision, commit)
        header = HEADER.format(label=label, config_id=config_id, commit=ns.commit, branch=ns.branch, datums_revision=revision,
                               openarm_description=commit, platform_sw=ns.platform_sw)
        assert "--" not in header[len("<!--"):-len("-->")], "a double hyphen is not allowed inside an XML comment"
        text, n = BANNER_RE.subn(lambda m: m.group(1) + header, urdf, count=1)
        if n != 1:
            raise SystemExit(f"{label}: the xacro banner was not where expected, so nothing was rewritten; not guessing")
        filename = f"{mast_id}__{arm_package}__{hardware_profile}.urdf"
        with open(os.path.join(ns.out, filename), "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
        entries.append({"label": label, "mast_id": mast_id, "arm_package": arm_package, "hardware_profile": hardware_profile,
                        "file": filename, "config_id": config_id, "bytes": len(text.encode("utf-8")), **summary(text)})
    doc = {"schema": "openamr.frozen_expansions.v1", "source_commit": ns.commit, "source_branch": ns.branch,
           "datums_revision": revision, "openarm_description_commit": commit, "openarm_description_source": source,
           "openamr_platform_sw_commit": ns.platform_sw, "entries": entries}
    with open(os.path.join(ns.out, "index.partial.json"), "w") as f:
        json.dump(doc, f, indent=2)
        f.write("\n")
    print(f"{len(entries)} expansions written to {ns.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
