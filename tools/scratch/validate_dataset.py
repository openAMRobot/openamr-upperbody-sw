# SPDX-License-Identifier: MIT
"""Scratch (not for merge): reopen a converted LeRobot dataset with the native loader and run the C9/C10 checks.

Reports episodes, frames, fps, features, camera keys, unit and range checks per channel, the converter manifest's
equal-timestamp drops, pre-origin frames, held commands, tolerance flags and lift-phase evidence, and the raw C10
decision (no policy: nothing is approved). Simulation evidence only.
"""
import argparse
import collections
import importlib.metadata
import json
import math
from pathlib import Path


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--dataset", type=Path, required=True)
    p.add_argument("--profile", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args()
    import numpy as np
    from jsonschema import Draft202012Validator
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    from oa_embodied_ai.conversion.native import LEROBOT_VERSION
    from oa_embodied_ai.dataset_qa.contract import load_c9_schema
    from oa_embodied_ai.dataset_qa.episode import check_dataset

    root = a.dataset
    profile = json.loads(a.profile.read_text())
    info = json.loads((root / "meta/info.json").read_text())
    manifest = json.loads((root / "meta/openamrobot/conversion.json").read_text())
    out = {"lerobot_installed": importlib.metadata.version("lerobot"), "lerobot_pinned_by_converter": LEROBOT_VERSION}
    ds = LeRobotDataset(info["dataset_id"], root=root, video_backend="pyav")
    cams = sorted(k for k in info["features"] if k.startswith("observation.images."))
    decoded = 0
    shapes = {}
    for i in range(len(ds)):
        row = ds[i]
        for k in cams:
            shapes.setdefault(k, list(row[k].shape))
        decoded += 1
    hf = ds.hf_dataset
    state = np.stack([np.asarray(v, dtype=np.float64) for v in hf["observation.state"]])
    action = np.stack([np.asarray(v, dtype=np.float64) for v in hf["action"]])
    ts = [float(v) for v in hf["timestamp"]]
    out.update(
        loader="lerobot.datasets.lerobot_dataset.LeRobotDataset (video_backend pyav)",
        episodes=ds.num_episodes, frames=ds.num_frames, frames_decoded_with_video=decoded, fps=ds.fps,
        features=sorted(info["features"]), camera_keys=cams, camera_shapes_chw=shapes,
        timestamps={"first_s": ts[0], "last_s": ts[-1],
                    "on_grid": all(abs(t - i / ds.fps) < 1e-4 for i, t in enumerate(ts))})
    sidecar = json.loads(sorted((root / "meta/openamrobot/episodes").glob("episode-*.json"))[0].read_text())
    schema, provenance = load_c9_schema()
    errors = [e.message for e in Draft202012Validator(schema).iter_errors(sidecar)]
    out["c9"] = {"schema": provenance, "schema_errors": errors[:5], "valid": not errors,
                 "action_semantics": sidecar["openamrobot"]["action_semantics"],
                 "recording": sidecar["openamrobot"]["recording"],
                 "data_origin": sidecar["openamrobot"]["provenance"]["data_origin"]}
    units = {}
    for kind, values, key in (("state", state, "observation.state"), ("action", action, "action")):
        dims = sidecar["native_lerobot"]["features"][key]["dimensions"]
        for i, (c, d) in enumerate(zip(profile[kind + "_channels"], dims, strict=True)):
            lo, hi = float(values[:, i].min()), float(values[:, i].max())
            rng, tol = c.get("range"), c.get("range_tolerance", 0.0)
            entry = {"unit": d["unit"], "profile_unit": c["unit"], "meaning": d["meaning"], "min": lo, "max": hi,
                     "unit_matches_profile": d["unit"] == c["unit"],
                     "angular_within_2pi": (max(abs(lo), abs(hi)) <= 2 * math.pi) if c["unit"] in ("rad", "rad/s") else None}
            if rng is not None:
                entry.update(range=rng, within_range=rng[0] <= lo and hi <= rng[1],
                             within_range_and_tolerance=rng[0] - tol <= lo and hi <= rng[1] + tol)
            units[f"{kind}.{c['name']}"] = entry
    out["units"] = units
    out["units_summary"] = {
        "all_units_match_profile": all(e["unit_matches_profile"] for e in units.values()),
        "angular_all_within_2pi": all(e["angular_within_2pi"] in (True, None) for e in units.values()),
        "outside_range_but_within_tolerance": sorted(k for k, e in units.items()
                                                     if e.get("within_range") is False and e.get("within_range_and_tolerance")),
        "outside_range_and_tolerance": sorted(k for k, e in units.items() if e.get("within_range_and_tolerance") is False)}
    ep = manifest["episodes"][0]
    out["manifest"] = {k: manifest.get(k) for k in ("origin", "profile_status", "profile_registered", "proposed_rules_enabled",
                                                    "proposed_rules_note", "training_readiness", "physical_acceptance",
                                                    "native_loader")}
    out["equal_timestamp_dropped"] = {s: v["equal_timestamp_dropped"] for s, v in ep["stream_timestamps"].items()}
    out["messages"] = {s: v["messages"] for s, v in ep["stream_timestamps"].items()}
    out["alignment"] = ep["alignment"]
    lift = dict(ep["lift_phase"])
    out["lift_phase"] = lift
    report = check_dataset(root)
    counts = collections.Counter((f["check_id"], f["status"]) for f in report["findings"])
    by_status = collections.Counter(f["status"] for f in report["findings"])
    out["c10"] = {"status": report["status"], "exit_code": report["exit_code"], "training_ready": report["training_ready"],
                  "findings_by_status": dict(by_status),
                  "findings_by_check": {f"{c}:{s}": n for (c, s), n in sorted(counts.items())},
                  "rejects": [{k: f.get(k) for k in ("check_id", "frame", "stream", "observed", "expected")}
                              for f in report["findings"] if f["status"] == "REJECT"][:10],
                  "policy": "none supplied (no approved policy exists)"}
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / "c10-report.json").write_text(json.dumps(report, indent=1, default=str))
    (a.out / "validation.json").write_text(json.dumps(out, indent=1, default=str))
    print("=== VALIDATION JSON")
    print(json.dumps(out, default=str))
    print("=== END")
    ok = (out["episodes"] == 1 and out["frames"] == decoded and out["c9"]["valid"] and not out["c10"]["rejects"]
          and out["lerobot_installed"] == LEROBOT_VERSION)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
