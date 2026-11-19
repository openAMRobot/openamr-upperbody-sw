# SPDX-License-Identifier: MIT
"""Scratch (not for merge): print the reference manifest summary of a converted dataset (no per-frame lists)."""
import json
import sys
from pathlib import Path

dataset, *verifies = sys.argv[1:]
m = json.loads((Path(dataset) / "meta/openamrobot/conversion.json").read_text())
e = m["episodes"][0]
r = e["reference"]
checks = r["checks"]
summary = {
    "converter": {k: m["execution"].get(k) for k in ("git_commit", "source_tree_clean", "source_sha256", "dependencies")},
    "configuration": {k: v for k, v in m["reference"]["configuration"].items() if k != "sidecar_templates"},
    "source": r["source"],
    "robot_model": r["robot_model"],
    "calibration": r["calibration"],
    "not_available": r["not_available"],
    "timestamps": checks["timestamps"],
    "actions": checks["actions"],
    "cameras": {k: v for k, v in checks["cameras"].items() if k != "per_frame_max_skew_ns"},
    "gripper": checks["gripper"],
    "alignment": {k: v for k, v in e["alignment"].items() if k != "range_tolerance_flags"},
    "range_tolerance_flags": e["alignment"].get("range_tolerance_flags", {}).get("exported_frames"),
    "output_files": len(m["reference"]["output_sha256"]),
    "output_sha256": m["reference"]["output_sha256"],
    "verify": [{k: json.loads(Path(v).read_text()).get(k) for k in ("status", "dataset_sha256", "manifest_sha256", "checks")} for v in verifies],
}
print(json.dumps(summary, indent=1, sort_keys=True))
