# SPDX-License-Identifier: MIT
"""Scratch (not for merge): check downloaded release assets against the dataset's own manifest.

Usage: verify_dev_evidence.py DATASET_DIR BAG_DIR
Exits non-zero on any mismatch. Standard library only.
"""

import hashlib
import json
import sys
from pathlib import Path

CONVERSION_JSON = "meta/openamrobot/conversion.json"
EXPECTED = {
    CONVERSION_JSON: "2d0855f61daac7e8d65c6844cb3c94e87bd5e43daf823eeb14967106d445b1db",
    "bag_0.mcap": "4b1f23f220b1b8aaedeff6da34e76131aee62373399683bd083087418459bf50",
    "metadata.yaml": "cf2a9bb5fc3dcfc0476a6fb1372df863e2ae956f137e456c8cf327e829d27f47",
}
FRAMES = 543


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def bare(value):
    return value.split(":", 1)[1] if value.startswith("sha256:") else value


def files(root):
    return {p.relative_to(root).as_posix(): sha256(p) for p in sorted(root.rglob("*")) if p.is_file()}


def main():
    dataset, bag = Path(sys.argv[1]), Path(sys.argv[2])
    failures = []

    def check(name, ok, detail):
        print(("PASS " if ok else "FAIL ") + name + ": " + detail)
        if not ok:
            failures.append(name)

    got = sha256(dataset / CONVERSION_JSON)
    check("conversion.json", got == EXPECTED[CONVERSION_JSON], got)
    manifest = json.loads((dataset / CONVERSION_JSON).read_text())

    actual = files(dataset)
    actual.pop(CONVERSION_JSON)
    expected = {k: bare(v) for k, v in manifest["reference"]["output_sha256"].items()}
    diff = sorted(k for k in set(actual) | set(expected) if actual.get(k) != expected.get(k))
    check("dataset files vs reference.output_sha256", not diff, f"{len(actual)} files, differing: {diff}")

    bag_actual = files(bag)
    bag_expected = {k: bare(v) for k, v in manifest["episodes"][0]["source_bag_sha256"].items()}
    check("bag files vs source_bag_sha256", bag_actual == bag_expected, json.dumps(bag_actual))
    for name in ("bag_0.mcap", "metadata.yaml"):
        check(name, bag_actual.get(name) == EXPECTED[name], str(bag_actual.get(name)))

    frames = json.loads((dataset / "meta/info.json").read_text()).get("total_frames")
    check("total_frames", frames == FRAMES, str(frames))

    if failures:
        print("VERIFY FAILED: " + ", ".join(failures))
        return 1
    print("VERIFY PASS: every asset matches the manifest")
    return 0


if __name__ == "__main__":
    sys.exit(main())
