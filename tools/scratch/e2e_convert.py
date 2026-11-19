# SPDX-License-Identifier: MIT
"""Scratch (not for merge): run the embodied-ai converter's own stages on a recorded Baseline 1 bag.

1. Full 19-channel Proposed profile, development copy (origin synthetic, declared simulation): each converter stage
   is run in order and its PASS or REFUSE recorded; a refusal is reported, never worked around.
2. Gripper subset (both grippers, four cameras), development copy: oa-convert-bag writes a LeRobot dataset, then the
   LeRobot loader opens it and the counts, units and timestamps are checked.
Simulation evidence only. Nothing here registers or accepts a profile.
"""
import argparse
import copy
import hashlib
import json
import subprocess
import sys
import traceback
from pathlib import Path


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def development_copy(profile, contract):
    dev = copy.deepcopy(profile)
    dev["origin"] = "synthetic"
    dev["provenance"]["origin_declaration"] = (
        "DEVELOPMENT COPY of a Proposed simulation profile: converted through the converter's synthetic-development "
        "path because the simulation profile is not registered. Data: Gazebo simulation run 44448676604, never robot data")
    if contract:
        dev["component_contract"] = {"path": str(contract), "sha256": sha256(contract)}
    return dev


def stage(results, name, fn):
    try:
        value = fn()
        results.append({"stage": name, "status": "PASS"})
        return value
    except Exception as exc:  # report every refusal with its type
        results.append({"stage": name, "status": "REFUSED", "error": f"{type(exc).__name__}: {exc}"})
        return None


def census(bag, profile, out):
    from oa_embodied_ai.recording.bag import align, read_bag, validate_recording_phases, validate_source_samples
    from oa_embodied_ai.recording.profile import validate_profile

    results = []
    report = {}
    stage(results, "validate_profile(development=True)", lambda: validate_profile(profile, development=True))
    data = stage(results, "read_bag", lambda: read_bag(bag, profile, report=report))
    if data is not None:
        stage(results, "validate_recording_phases", lambda: validate_recording_phases(data, profile))
        # validate_source_samples per state/action stream, so every refusing stream is named, not only the first
        for kind in ("state", "action"):
            for stream in dict.fromkeys(c["stream"] for c in profile[kind + "_channels"]):
                sub = copy.deepcopy(profile)
                sub["state_channels"] = [c for c in profile["state_channels"] if kind == "state" and c["stream"] == stream]
                sub["action_channels"] = [c for c in profile["action_channels"] if kind == "action" and c["stream"] == stream]
                sub["streams"] = [s for s in profile["streams"] if s["id"] == stream]
                stage(results, f"validate_source_samples {kind} {stream}", lambda sub=sub: validate_source_samples(data, sub))
        stage(results, "align", lambda: align(data, profile))
        from oa_embodied_ai.recording.bag import extract
        ranges = {}
        for kind in ("state", "action"):
            for c in profile[kind + "_channels"]:
                vals = []
                for smp in data[c["stream"]]:
                    try:
                        vals.append(extract(smp, c))
                    except Exception as exc:
                        ranges[f"{kind}.{c['name']}"] = {"extract_error": str(exc)}
                        break
                if vals:
                    lim = c.get("range")
                    out = [v for v in vals if lim and not lim[0] <= v <= lim[1]]
                    ranges[f"{kind}.{c['name']}"] = {"stream": c["stream"], "n": len(vals), "min": min(vals), "max": max(vals),
                                                      "range": lim, "outside": len(out),
                                                      "worst": (min(out, key=lambda v: min(abs(v - lim[0]), abs(v - lim[1])) * -1) if out else None)}
        frames = {}
        for st in profile["streams"]:
            hdr = [getattr(getattr(x.message, "header", None), "frame_id", None) for x in data[st["id"]][:50]]
            frames[st["id"]] = sorted({str(h) for h in hdr})
        results.append({"stage": "value ranges per channel (source samples)", "ranges": ranges})
        results.append({"stage": "recorded header frame_id per stream (first 50)", "frames": frames})
    for kind in ("state", "action"):
        sources = sorted({c["stream"] for c in profile[kind + "_channels"]})
        results.append({"stage": f"sidecar single acquisition source ({kind})",
                         "status": "PASS" if len(sources) == 1 else "REFUSED",
                         "sources": sources})
    (out / "census.json").write_text(json.dumps({"stages": results, "stream_timestamps": report}, indent=1))
    return results, report


def subset_profile(profile):
    sub = copy.deepcopy(profile)
    sub["profile_id"] = "dev-baseline1-gripper-subset"
    sub["episode_id"] = "dev-baseline1-gripper-subset-run-44448676604"
    sub["dataset_id"] = "openamrobot/dev-baseline1-gripper-subset"
    sub["task"] = profile["task"] + " (development subset: grippers and four cameras only)"
    sub["components"] = ["left_gripper", "right_gripper"]
    sub["state_channels"] = [c for c in profile["state_channels"] if c["component"] in sub["components"]]
    sub["action_channels"] = [c for c in profile["action_channels"] if c["component"] in sub["components"]]
    for c in sub["state_channels"]:
        c["stream"] = "arm_joint_states"
    used = {c["stream"] for c in sub["state_channels"] + sub["action_channels"]}
    sub["streams"] = [s for s in profile["streams"] if s.get("camera_key") or s["id"] in used]
    sub["state_names"] = [c["name"] for c in sub["state_channels"]]
    sub["action_names"] = [c["name"] for c in sub["action_channels"]]
    sub.pop("component_contract", None)
    sub["phase_evidence"] = {}
    return sub


def load_and_check(root):
    import numpy as np
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    info = json.loads((Path(root) / "meta/info.json").read_text())
    d = LeRobotDataset(info.get("dataset_id", "openamrobot/dev"), root=root, video_backend="pyav")
    names_state = info["features"]["observation.state"]["names"]
    names_action = info["features"]["action"]["names"]
    state = np.stack([np.asarray(d.hf_dataset[i]["observation.state"], dtype=float) for i in range(len(d))]) if len(d) else None
    action = np.stack([np.asarray(d.hf_dataset[i]["action"], dtype=float) for i in range(len(d))]) if len(d) else None
    ts = [float(d.hf_dataset[i]["timestamp"]) for i in range(len(d))]
    ep = [int(d.hf_dataset[i]["episode_index"]) for i in range(len(d))]
    monotonic = all((e2 != e1) or (t2 > t1) for (t1, e1), (t2, e2) in zip(zip(ts, ep), zip(ts[1:], ep[1:])))
    units = {}
    for names, values, kind in ((names_state, state, "state"), (names_action, action, "action")):
        for i, n in enumerate(names):
            lo, hi = (0.0, 0.7854) if "left_finger" in n else (-0.7854, 0.0)
            units[f"{kind}.{n}"] = {"min": float(values[:, i].min()), "max": float(values[:, i].max()),
                                    "model_range_rad": [lo, hi],
                                    "within": bool(values[:, i].min() >= lo - 1e-6 and values[:, i].max() <= hi + 1e-6),
                                    "abs_within_0_0.785": bool(np.abs(values[:, i]).max() <= 0.7854 + 1e-6)}
    first = d[0]
    cams = sorted(k for k in info["features"] if k.startswith("observation.images."))
    sidecars = sorted((Path(root) / "meta/openamrobot/episodes").glob("episode-*.json"))
    side_mono = []
    for path in sidecars:
        s = json.loads(path.read_text())
        a = [r["authoritative_timestamp_ns"] for r in s["openamrobot"]["samples"]]
        side_mono.append(all(y > x for x, y in zip(a, a[1:])))
    conversion = json.loads((Path(root) / "meta/openamrobot/conversion.json").read_text())
    return {
        "lerobot_loader": "LeRobotDataset",
        "episodes": d.num_episodes, "frames": d.num_frames, "len": len(d), "fps": d.fps,
        "features": sorted(info["features"]), "camera_keys": cams,
        "camera_shapes_first_frame": {k: list(first[k].shape) for k in cams},
        "state_names": names_state, "action_names": names_action,
        "units_check": units,
        "timestamps": {"first": ts[0], "last": ts[-1], "monotonic_per_episode": monotonic,
                       "c9_sidecar_authoritative_monotonic": side_mono},
        "converter_manifest": {k: conversion.get(k) for k in ("origin", "training_readiness", "physical_acceptance", "native_loader")},
        "stream_timestamps": [e.get("stream_timestamps") for e in conversion.get("episodes", [])],
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--bag", type=Path, required=True)
    p.add_argument("--profile", type=Path, required=True)
    p.add_argument("--contract", type=Path, required=True)
    p.add_argument("--template", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    proposed = json.loads(a.profile.read_text())
    full = development_copy(proposed, a.contract)
    (a.out / "full_dev_profile.json").write_text(json.dumps(full, indent=1))
    results, report = census(a.bag, full, a.out)
    print("=== FULL 19-CHANNEL CENSUS")
    print(json.dumps({"stages": results, "stream_timestamps": report}))
    attempts = []
    for fps, skew in ((5, 100_000_000),):
        sub = subset_profile(full)
        sub["fps"], sub["max_skew_ns"] = fps, skew
        path = a.out / f"subset_profile_fps{fps}.json"
        path.write_text(json.dumps(sub, indent=1))
        target = a.out / f"dataset_fps{fps}"
        cmd = ["oa-convert-bag", str(a.bag), "--profile", str(path), "--sidecar-template", str(a.template),
               "--output", str(target), "--synthetic-development"]
        r = subprocess.run(cmd, capture_output=True, text=True)
        attempt = {"fps": fps, "max_skew_ns": skew, "command": " ".join(cmd), "exit": r.returncode,
                   "stdout": r.stdout.strip()[-2000:], "stderr": r.stderr.strip()[-3000:]}
        if r.returncode == 0:
            try:
                attempt["loader"] = load_and_check(target)
            except Exception:
                attempt["loader_error"] = traceback.format_exc()[-3000:]
        attempts.append(attempt)
    print("=== SUBSET CONVERSION")
    print(json.dumps(attempts))
    (a.out / "subset.json").write_text(json.dumps(attempts, indent=1))
    return 0 if any(x.get("loader") for x in attempts) else 1


if __name__ == "__main__":
    sys.exit(main())
