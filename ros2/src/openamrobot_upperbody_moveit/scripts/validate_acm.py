#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""M03: validate the SRDF's collision matrix. Every disabled pair needs a reason that can be shown; sampling is not one.

    ros2 run openamrobot_upperbody_moveit validate_acm.py [--samples 2000] [--seed N] [--lift-stop L1000 ...] [--srdf PATH] [--json OUT]

A pair may be disabled for exactly three reasons (acm_derive.py has the definitions): declared by upstream (section 1 of the SRDF), `Adjacent` (a
structural neighbour) or `ConstantTransform` (no movable joint between the links, so their relative transform is a constant and ONE evaluation
settles every configuration). Everything else is enabled. This check recomputes the reasons from the model and fails:

  * a disabled pair that none of the three reasons covers (a pair somebody disabled because it "never collides");
  * a pair whose `reason` label is not true (`Adjacent` that is not a structural neighbour or upstream's, `ConstantTransform` that has a movable
    joint between its links, any other label, including the old sampled `Never`);
  * a disabled `ConstantTransform` pair that collides in the one evaluation (the claim is that it is clear), and as a check on that reasoning, any
    disagreement between that evaluation and the first 200 sampled states;
  * a constant pair that is NOT disabled and collides: a permanent collision, every state of the model invalid.

There is no WARN class: a disabled pair that is wrong is a FAIL. `Adjacent` and upstream-declared pairs touch by design and are not claims about
collision. The sampled part is INFORMATION about the ENABLED pairs: N random joint states inside the limits (numpy default_rng(seed), the same
states at every lift stop), and the pairs found colliding, with how often. A pair that collides often even though the matrix is right tells A4
something about reach and clearance. Sampling falsifies and never certifies, and nothing in the matrix rests on it.

Each lift stop (a calibration ID: lift_joint held at that stop's position, every other joint sampled) runs in its own process (a moveit_py instance does not shut down cleanly, and the process leaves with os._exit after writing its
result). Exit status: 1 if the matrix is wrong, 2 if the check could not run, else 0.
"""
import argparse
import itertools
import json
import math
import os
import re
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

LIFT_JOINT = "lift_joint"
STOPS = ("L1000", "L1175", "L1350")        # option B's calibration stops (upperbody_configuration.yaml lift_calibration_stops)
DEFAULT_SAMPLES = 2000
DEFAULT_SEED = 20261006
CROSS_CHECK_SAMPLES = 200       # states on which the constant pairs are re-evaluated, to test the reasoning (not to certify)
BEGIN_MARKER = "<!-- BEGIN GENERATED ACM -->"
TOP = 25                        # how many of the most often colliding enabled pairs the summary lists


def pair_key(a, b):
    return tuple(sorted((a, b)))


def srdf_pairs(path):
    """{pair: reason} for every disable_collisions of an SRDF file."""
    root = ET.parse(path).getroot()
    return {pair_key(d.get("link1"), d.get("link2")): d.get("reason") for d in root.findall("disable_collisions")}


def srdf_imported_pairs(path):
    """{pair: reason} disabled in the SRDF before the derived block: section 1, copied from upstream's v2.0 SRDF (upstream's own declaration)."""
    with open(path) as f:
        text = f.read()
    head = text.split(BEGIN_MARKER)[0] + "</robot>"
    return {pair_key(d.get("link1"), d.get("link2")): d.get("reason") for d in ET.fromstring(head).findall("disable_collisions")}


def joint_table(urdf_text):
    """({joint: (lower, upper)} for the joints that are sampled, {mimic joint: (leader, multiplier, offset)})."""
    root = ET.fromstring(urdf_text)
    free, mimic = {}, {}
    for joint in root.findall("joint"):
        kind, name = joint.get("type"), joint.get("name")
        if kind == "fixed":
            continue
        m = joint.find("mimic")
        if m is not None:
            mimic[name] = (m.get("joint"), float(m.get("multiplier", 1.0)), float(m.get("offset", 0.0)))
        elif kind == "continuous":
            free[name] = (-math.pi, math.pi)
        else:
            limit = joint.find("limit")
            free[name] = (float(limit.get("lower")), float(limit.get("upper")))
    return free, mimic


def sample_states(free, mimic, count, seed):
    """`count` joint states, each {joint: value}, uniform inside the limits (continuous joints in [-pi, pi]); a mimic joint follows its
    leader exactly as the URDF says. numpy's default_rng(seed): the same states at every run and every lift stop."""
    import numpy as np
    rng = np.random.default_rng(seed)
    names = sorted(free)
    lows = np.array([free[n][0] for n in names])
    highs = np.array([free[n][1] for n in names])
    for values in rng.uniform(lows, highs, size=(count, len(names))):
        state = {n: float(v) for n, v in zip(names, values)}
        for joint, (leader, multiplier, offset) in mimic.items():
            state[joint] = state[leader] * multiplier + offset
        yield state


def check_matrix(urdf_root, pairs, upstream):
    """The static half of M03, no collision checker needed: -> {unjustified, bad_labels, adjacent, constant} for the matrix `pairs`
    ({pair: reason}) against the model, with `upstream` the pairs section 1 declares."""
    from openamrobot_upperbody_moveit.acm_derive import ADJACENT, CONSTANT, REASONS, constant_pairs, justifications, structural_adjacent
    adjacent, constant = structural_adjacent(urdf_root), constant_pairs(urdf_root)
    links = {l.get("name") for l in urdf_root.findall("link")}
    unjustified, bad = [], []
    for pair, reason in sorted(pairs.items()):
        if not set(pair) <= links:
            bad.append({"pair": list(pair), "problem": "a link the model does not have"})
            continue
        if not justifications(pair, urdf_root, upstream, adjacent, constant):
            unjustified.append({"pair": list(pair), "reason_label": reason,
                                "problem": "no reason that can be shown: not declared by upstream, not a structural neighbour, no constant transform"})
        if reason not in REASONS:
            bad.append({"pair": list(pair), "problem": f"label {reason!r} is not one of {REASONS} (a sampled `Never` is not a reason)"})
        elif reason == ADJACENT and pair not in adjacent and pair_key(*pair) not in upstream:
            bad.append({"pair": list(pair), "problem": "labelled Adjacent but neither a structural neighbour nor declared by upstream"})
        elif reason == CONSTANT and pair not in constant:
            bad.append({"pair": list(pair), "problem": "labelled ConstantTransform but a movable joint lies between its links"})
    return {"unjustified": unjustified, "bad_labels": bad, "adjacent": adjacent, "constant": constant}


def sample_one_stop(stop, samples, seed, srdf_file, hardware_profile="mock"):
    """Run the check at one lift stop inside this process (moveit_py): lift_joint is held at the stop's position and everything else is
    sampled. Returns the result dict."""
    from openamrobot_upperbody_moveit.acm_derive import CONSTANT, geometry_links
    from openamrobot_upperbody_moveit.acm_scene import colliding_pairs, open_scene
    from openamrobot_upperbody_moveit.moveit_config import lift_stops
    lift_position = lift_stops()[stop]

    srdf_file = srdf_file or default_srdf()
    pairs = srdf_pairs(srdf_file)
    upstream = srdf_imported_pairs(srdf_file)
    started = time.monotonic()
    with open_scene(srdf_file=srdf_file, node_name="validate_acm", hardware_profile=hardware_profile) as scene:
        static = check_matrix(scene.urdf_root, pairs, upstream)
        constant, adjacent = static["constant"], static["adjacent"]
        free, mimic = joint_table(scene.urdf_text)
        free.pop(LIFT_JOINT)                    # held at the stop, not sampled: a stop is a calibration ID
        geometry = sorted(geometry_links(scene.urdf_root))
        disabled_constant = sorted(p for p, r in pairs.items() if r == CONSTANT)
        enabled_constant = sorted(constant - set(pairs))
        enabled = [pair_key(a, b) for a, b in itertools.combinations(geometry, 2) if pair_key(a, b) not in pairs]

        # (1) by construction: ONE evaluation, in the default state. A fixed relative transform gives the same answer in every state.
        scene.set_default_state()
        scene.set_joint_positions({LIFT_JOINT: lift_position})
        constant_colliding = sorted(scene.colliding(disabled_constant))
        permanent = sorted(scene.colliding(enabled_constant))
        cross_check = {"states": min(CROSS_CHECK_SAMPLES, samples), "disagreements": []}
        expected_clear = set(constant_colliding)

        # (2) information: sampling over the ENABLED pairs
        hits = {}
        for index, sample in enumerate(sample_states(free, mimic, samples, seed)):
            scene.set_joint_positions({**sample, LIFT_JOINT: lift_position})
            for pair in colliding_pairs(scene.any_collision, enabled):
                row = hits.setdefault(pair, {"count": 0, "first_sample_index": index})
                row["count"] += 1
            if index < cross_check["states"]:
                again = set(scene.colliding(disabled_constant))
                if again != expected_clear:
                    cross_check["disagreements"].append({"sample_index": index, "differs": sorted(map(list, again ^ expected_clear))})
    elapsed = time.monotonic() - started
    reasons = {}
    for reason in pairs.values():
        reasons[reason] = reasons.get(reason, 0) + 1
    ranked = sorted(hits.items(), key=lambda kv: (-kv[1]["count"], kv[0]))
    failing = bool(static["unjustified"] or static["bad_labels"] or constant_colliding or permanent or cross_check["disagreements"])
    return {
        "lift_stop": stop, "lift_joint_position_m": lift_position, "samples": samples, "seed": seed, "seconds": round(elapsed, 1), "srdf": srdf_file,
        "disabled_pairs": {"total": len(pairs), "by_label": reasons, "declared_by_upstream": len(upstream),
                           "structural_adjacent_derived": len(adjacent), "constant_transform_disabled": len(disabled_constant)},
        "enabled_pairs": {"total": len(enabled), "of_geometry_bearing_links": len(geometry)},
        "unjustified": static["unjustified"], "bad_labels": static["bad_labels"],
        "constant_transform": {"evaluated": "once, in the default state at the stop", "pairs": [list(p) for p in disabled_constant],
                               "colliding": [list(p) for p in constant_colliding],
                               "cross_check_with_sampled_states": cross_check},
        "enabled_constant_pairs_colliding": [list(p) for p in permanent],
        "enabled_pairs_sampled": {"samples": samples, "seed": seed, "colliding_pairs": len(hits), "never_collided": len(enabled) - len(hits),
                                  "most_often": [{"pair": list(p), "collided_in_samples": r["count"], "of": samples,
                                                  "first_sample_index": r["first_sample_index"]} for p, r in ranked[:TOP]]},
        "result": "FAIL" if failing else "PASS",
    }


def default_srdf():
    from openamrobot_upperbody_moveit.moveit_config import srdf_path
    return srdf_path()


def run_one(args):
    result = sample_one_stop(args.lift_stop[0], args.samples, args.seed, args.srdf)
    with open(args.json, "w") as f:
        json.dump(result, f)
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)           # moveit_py segfaults when the interpreter tears it down; the result is already on disk


def print_stop(result):
    d, e = result["disabled_pairs"], result["enabled_pairs"]
    print(f"\n{result['lift_stop']} (lift_joint {result['lift_joint_position_m']:.3f} m): {d['total']} pairs disabled ({', '.join(f'{n} {k}' for k, n in sorted(d['by_label'].items()))}; "
          f"{d['declared_by_upstream']} declared by upstream), {e['total']} enabled; {result['seconds']} s")
    for item in result["unjustified"]:
        print(f"  FAIL: {item['pair'][0]} <-> {item['pair'][1]} is disabled with no reason that can be shown ({item['reason_label']})")
    for item in result["bad_labels"]:
        print(f"  FAIL: {item['pair'][0]} <-> {item['pair'][1]}: {item['problem']}")
    for a, b in result["constant_transform"]["colliding"]:
        print(f"  FAIL: {a} <-> {b} is disabled as ConstantTransform (clear in every configuration) and collides in the one evaluation")
    for item in result["constant_transform"]["cross_check_with_sampled_states"]["disagreements"]:
        print(f"  FAIL: the constant-transform reasoning is wrong: sample {item['sample_index']} differs from the one evaluation on {item['differs']}")
    for a, b in result["enabled_constant_pairs_colliding"]:
        print(f"  FAIL: {a} <-> {b} has a constant transform, is NOT disabled, and collides: a permanent collision, every state is invalid")
    if result["result"] == "PASS":
        print(f"  PASS: every disabled pair has a reason that can be shown; the {len(result['constant_transform']['pairs'])} constant-transform pairs are "
              "clear in the one configuration that decides them")
    s = result["enabled_pairs_sampled"]
    print(f"  information, not a result: of {e['total']} enabled pairs, {s['colliding_pairs']} collided in some of {s['samples']} samples at seed {s['seed']} "
          f"and {s['never_collided']} in none (that says nothing about whether they can)")
    for item in s["most_often"][:8]:
        print(f"    {item['collided_in_samples']:>5} of {item['of']}  {item['pair'][0]} <-> {item['pair'][1]}")


def run_all(args):
    stops = args.lift_stop or list(STOPS)
    results = []
    with tempfile.TemporaryDirectory() as tmp:
        for stop in stops:
            out = os.path.join(tmp, f"{stop}.json")
            command = [sys.executable, os.path.abspath(__file__), "--worker", "--lift-stop", stop, "--samples", str(args.samples),
                       "--seed", str(args.seed), "--json", out]
            if args.srdf:
                command += ["--srdf", args.srdf]
            done = subprocess.run(command, capture_output=True, text=True, env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"})
            if done.returncode != 0 or not os.path.isfile(out):
                sys.stderr.write(done.stdout[-2000:] + done.stderr[-2000:])
                print(f"validate_acm: the check at {stop} could not run (exit {done.returncode})", file=sys.stderr)
                return 2, results
            with open(out) as f:
                results.append(json.load(f))
            print_stop(results[-1])
    return (1 if any(r["result"] == "FAIL" for r in results) else 0), results


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--samples", type=int, default=DEFAULT_SAMPLES, help="random joint states per lift stop for the information sampling")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument("--lift-stop", action="append", choices=STOPS, help="repeatable; default all three")
    parser.add_argument("--srdf", default=None, help="an SRDF to check instead of the installed one (the mutation tests use this)")
    parser.add_argument("--json", default=None, help="write the result here (one lift stop: its result; several: the summary)")
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)     # how this script runs one lift stop in a process of its own
    args = parser.parse_args(argv)
    if args.worker:
        return run_one(args)
    status, results = run_all(args)
    summary = {"samples_per_lift_stop": args.samples, "seed": args.seed, "results": results,
               "result": "ERROR" if status == 2 else ("FAIL" if status == 1 else "PASS")}
    if args.json:
        with open(args.json, "w") as f:
            json.dump(summary, f, indent=1)
    print(f"\nM03: {summary['result']}  ({len(results)} lift stop(s); information sampling {args.samples} states at seed {args.seed})")
    return status


if __name__ == "__main__":
    sys.exit(main())
