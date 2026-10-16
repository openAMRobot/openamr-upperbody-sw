#!/usr/bin/env python3
"""Derive the collision matrix of config/openamrobot.srdf from the model, and propose it (or, with --write, replace the contract).

    ros2 run openamrobot_upperbody_moveit derive_acm.py [--base-plate bp000] [--srdf PATH] [--write | --proposal PATH] [--json OUT]

A pair is disabled for exactly three reasons (acm_derive.py has the definitions): declared by upstream (section 1 of the SRDF, copied verbatim and
checked against upstream's own file by test_srdf_import.py), `Adjacent` (a structural neighbour), `ConstantTransform` (no movable joint between the
links, so one evaluation in the default state settles every configuration, and only pairs found CLEAR there are disabled). Everything else is
enabled. Nothing is sampled and nothing is seeded, so deriving twice gives the same matrix; the script says so by printing the fingerprint.

The checked-in SRDF is the contract. Without --write this writes the derived SRDF to --proposal (default: next to this run's temporary files) and
prints what differs from the contract; with --write it replaces the SRDF's generated block and rewrites config/acm/acm_generation.yaml, fingerprint
included, and the commit that does it is a contract change. A constant pair that collides and is neither Adjacent nor upstream-declared is a
permanent collision of the model: it is reported, nothing is written, and the exit status is 1.

Runs moveit_py (the one evaluation), so it leaves with os._exit after printing: moveit_py segfaults when it is torn down.
"""
import argparse
import datetime
import os
import re
import sys
import tempfile
import xml.etree.ElementTree as ET

import yaml

BEGIN, END = "<!-- BEGIN GENERATED ACM -->", "<!-- END GENERATED ACM -->"
DEFAULT_BASE_PLATE = "bp000"        # option B's baseline; the matrix does not depend on it (the plate moves the column fore-aft)


def split_srdf(text):
    match = re.search(re.escape(BEGIN) + r".*?" + re.escape(END), text, flags=re.S)
    if not match:
        raise SystemExit(f"derive_acm: {BEGIN} ... {END} not found in the SRDF")
    return text[:match.start()], text[match.end():]


def block_text(derived, counts, base_plate):
    lines = [BEGIN,
             f"    <!-- Derived by scripts/derive_acm.py (P12A) from the model's kinematic tree at {base_plate}, lift_joint at 0: no sampling, no seed. A pair is",
             "         disabled for exactly three reasons, upstream-declared (section 1), `Adjacent` (a structural neighbour) and",
             f"         `ConstantTransform` (no movable joint between the links, clear in the one evaluation that settles every configuration):",
             f"         {counts['adjacent']} Adjacent and {counts['constant_transform']} ConstantTransform pairs here, none of them from a sampler.",
             "         Every other pair is enabled. Record: config/acm/acm_generation.yaml; the way to redo it: scripts/regenerate_acm.sh. -->"]
    for (a, b), reason in sorted(derived.items()):
        lines.append(f'    <disable_collisions link1="{a}" link2="{b}" reason="{reason}"/>')
    lines.append("    " + END)
    return "\n".join(lines)


def contract_record(matrix, report, base_plate, moveit_version):
    from openamrobot_upperbody_moveit.acm_split import acm_fingerprint
    reasons = {}
    for reason in matrix.values():
        reasons[reason] = reasons.get(reason, 0) + 1
    return {
        "schema_version": 2,
        "method": "derived: scripts/derive_acm.py over the model's kinematic tree and upstream's own SRDF; no sampling, no seed (P12A)",
        "date": datetime.date.today().isoformat(), "base_plate": base_plate, "lift_joint_position_m": 0.0,
        "moveit_version": moveit_version,
        "rules": {
            "upstream_declared": "pairs upstream's SRDF states (enactic/openarm_ros2 @ 4e837e1), section 1 of the SRDF, verbatim",
            "Adjacent": "both links carry collision geometry, no other geometry-bearing link between them on the tree path, at most one movable joint",
            "ConstantTransform": "no movable joint between the links (one evaluation settles every configuration) and CLEAR in that evaluation",
            "everything_else": "enabled: sampling falsifies and never certifies, so it is not a reason to disable a pair"},
        "counts": {"upstream_declared": len(report["upstream"]), "adjacent_derived": len(report["adjacent"]),
                   "constant_transform": len(report["constant_transform"]), "disabled_total": len(matrix)},
        "deterministic": True,
        "contract": {"file": "config/openamrobot.srdf",
                     "what": "every <disable_collisions> of the SRDF (section 1, upstream's, and the derived block). This file is the contract: "
                             "M03 validates it, and a change to it is a change to the contract",
                     "fingerprint": acm_fingerprint(matrix),
                     "pairs": {"total": len(matrix), **reasons}},
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--base-plate", default=DEFAULT_BASE_PLATE, choices=("bp000", "bp050"))
    parser.add_argument("--srdf", help="the SRDF to take section 1 from and to compare with (default: the source tree's config/openamrobot.srdf)")
    parser.add_argument("--meta", help="config/acm/acm_generation.yaml (written only with --write)")
    parser.add_argument("--write", action="store_true", help="replace the contract: rewrite the SRDF's generated block and the record")
    parser.add_argument("--proposal", help="where the derived SRDF goes when --write is not given")
    parser.add_argument("--json", help="write the derivation report here")
    args = parser.parse_args(argv)

    from ament_index_python.packages import get_package_share_directory
    from openamrobot_upperbody_moveit.acm_derive import derive
    from openamrobot_upperbody_moveit.acm_scene import open_scene
    from openamrobot_upperbody_moveit.acm_split import acm_fingerprint, acm_pairs, diff_acm

    here = os.path.dirname(os.path.realpath(__file__))
    source = os.path.abspath(os.path.join(here, ".."))
    srdf_path = args.srdf or os.path.join(source, "config", "openamrobot.srdf")
    meta_path = args.meta or os.path.join(source, "config", "acm", "acm_generation.yaml")
    with open(srdf_path) as f:
        text = f.read()
    before, after = split_srdf(text)
    upstream = acm_pairs(ET.fromstring(before + after))
    current = acm_pairs(ET.fromstring(text))

    with open_scene(args.base_plate, srdf_file=srdf_path, node_name="derive_acm") as scene:
        scene.set_default_state()
        matrix, report = derive(scene.urdf_root, upstream, scene.colliding)
        version = ET.parse(os.path.join(get_package_share_directory("moveit_core"), "package.xml")).getroot().findtext("version")

    status = 0
    print(f"derived at {args.base_plate} (lift_joint at 0): {len(matrix)} disabled pairs = {len(report['upstream'])} declared by upstream + {len(report['adjacent'])} Adjacent "
          f"+ {len(report['constant_transform'])} ConstantTransform; every other pair is enabled")
    print(f"fingerprint {acm_fingerprint(matrix)}")
    if report["constant_and_colliding"]:
        status = 1
        print("FINDING: constant pairs that collide in every configuration and are neither Adjacent nor declared by upstream (a permanent collision "
              "of the model, not disabled, nothing written):")
        for a, b in report["constant_and_colliding"]:
            print(f"  {a} <-> {b}")
    added, removed, relabelled = diff_acm(current, matrix)
    print(f"against the contract ({len(current)} pairs): {len(added)} only in the derivation, {len(removed)} only in the contract, {len(relabelled)} relabelled")
    for title, items in (("only in the derivation", added), ("only in the contract", removed)):
        for (a, b), reason in sorted(items.items()):
            print(f"  {title}: {a} <-> {b} ({reason})")
    for (a, b), (was, now) in sorted(relabelled.items()):
        print(f"  relabelled: {a} <-> {b}: {was} -> {now}")

    if status == 0:
        derived_only = {p: r for p, r in matrix.items() if p not in upstream}
        merged = before + block_text(derived_only, {"adjacent": len(report["adjacent"]), "constant_transform": len(report["constant_transform"])},
                                     args.base_plate) + after
        if args.write:
            with open(srdf_path, "w") as f:
                f.write(merged)
            with open(meta_path, "w") as f:
                yaml.safe_dump(contract_record(matrix, report, args.base_plate, version), f, sort_keys=False, width=140)
            print(f"THE CONTRACT CHANGED (--write): {srdf_path} and {meta_path} rewritten")
        else:
            proposal = args.proposal or os.path.join(tempfile.mkdtemp(), "proposed_openamrobot.srdf")
            with open(proposal, "w") as f:
                f.write(merged)
            print(f"PROPOSAL written to {proposal}; the contract and its record are unchanged. To adopt it, re-run with --write and read the diff above.")
    if args.json:
        import json
        with open(args.json, "w") as f:
            json.dump({"base_plate": args.base_plate, "fingerprint": acm_fingerprint(matrix), "counts": {k: len(v) for k, v in report.items()},
                       "matrix": {f"{a} {b}": r for (a, b), r in sorted(matrix.items())}, "report": {k: [list(p) for p in v] for k, v in report.items()},
                       "diff": {"added": len(added), "removed": len(removed), "relabelled": len(relabelled)}}, f, indent=1)
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(status)


if __name__ == "__main__":
    main()
