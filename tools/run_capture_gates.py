#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Run openamrobot-embodied-ai's own capture gates, unchanged, on a candidate profile and a recorded bag (Proposed).

Calls oa_embodied_ai.recording.profile.validate_profile and oa_embodied_ai.recording.bag.read_bag from the pinned
embodied-ai checkout installed in the environment. Nothing is patched, registered or relaxed: a refusal is the
converter's answer and is reported as it is. Writes gates.json. Exit 0 when the report was written.
"""
import argparse
import json
import sys
from pathlib import Path


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--profile", required=True)
    p.add_argument("--bag", required=True)
    p.add_argument("--out", required=True)
    args = p.parse_args(argv)
    from oa_embodied_ai.conversion.validation import ConversionError
    from oa_embodied_ai.recording.bag import read_bag
    from oa_embodied_ai.recording.profile import validate_profile

    profile = json.loads(Path(args.profile).read_text())
    result = {}
    try:
        validate_profile(profile)
        result["validate_profile"] = {"status": "PASS"}
    except ConversionError as exc:
        result["validate_profile"] = {"status": "REFUSED", "reason": str(exc)}
    # read_bag checks topics, types, frame IDs, monotonic stamps and declared components; it does not need approval.
    try:
        data = read_bag(Path(args.bag), profile)
        result["read_bag"] = {"status": "PASS", "samples": {k: len(v) for k, v in data.items()}}
    except (ConversionError, ImportError) as exc:
        result["read_bag"] = {"status": "REFUSED", "reason": str(exc)}
    Path(args.out).write_text(json.dumps(result, indent=1) + "\n")
    print(json.dumps(result, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
