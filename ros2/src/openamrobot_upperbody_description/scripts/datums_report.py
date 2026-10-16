#!/usr/bin/env python3
"""Print datums.yaml grouped by status and by source, plus the placeholder list. Always exits 0 when the file reads.

    ros2 run openamrobot_upperbody_description datums_report.py [--datums PATH]

Reports; it does not judge. Profile `release` (fail on any placeholder) is a later step.
"""
import argparse
import os
import sys

import yaml

STATUS_ORDER = ["placeholder", "provisional", "verified"]
SOURCE_ORDER = ["measured", "contractor_m01", "f2s", "supplier", "plan", "upstream", "derived", "placeholder"]


def default_datums_path():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("openamrobot_upperbody_description"), "config", "datums.yaml")
    except Exception:  # not installed / no ament index: fall back to the source tree
        here = os.path.dirname(os.path.realpath(__file__))
        return os.path.join(here, "..", "config", "datums.yaml")


def _fmt(value):
    if value is None:
        return "null: HARD-FAIL (the expansion stops when this is read)"
    return str(value) if not isinstance(value, dict) else f"xyz={value['xyz']} rpy={value['rpy']}"


def _group(datums, key, order):
    groups = {}
    for name, d in datums.items():
        groups.setdefault(d[key], []).append(name)
    return [(k, sorted(groups[k])) for k in order + sorted(set(groups) - set(order)) if k in groups]


def report(doc, out=sys.stdout):
    datums = doc["datums"]
    w = max(len(n) for n in datums)
    print(f"datums.yaml revision {doc['revision']} (schema v{doc['schema_version']}): {len(datums)} datums", file=out)

    print("\nBy status", file=out)
    for status, names in _group(datums, "status", STATUS_ORDER):
        print(f"  {status:<12} {len(names):>2}", file=out)
        for n in names:
            print(f"      {n:<{w}}  [{datums[n]['source']}]  {_fmt(datums[n]['value'])}", file=out)

    print("\nBy source", file=out)
    for source, names in _group(datums, "source", SOURCE_ORDER):
        print(f"  {source:<15} {len(names):>2}  {', '.join(names)}", file=out)

    placeholders = sorted(n for n, d in datums.items() if d["status"] == "placeholder")
    print(f"\nPlaceholders ({len(placeholders)} of {len(datums)}): each needs a real value before a release profile", file=out)
    for n in placeholders:
        print(f"  {n:<{w}}  owner: {datums[n]['owner']}", file=out)

    report_only = doc.get("report_only", [])
    pending = doc.get("pending_reference", {})
    print(f"\nreport_only ({len(report_only)}): reported, never feeds the model", file=out)
    print("  " + (", ".join(report_only) or "-"), file=out)
    print(f"pending_reference ({len(pending)}): consumer not written yet", file=out)
    for n, step in pending.items():
        print(f"  {n:<{w}}  -> {step}", file=out)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--datums", default=None, help="datums.yaml (default: the installed package's)")
    args = parser.parse_args(argv)
    with open(args.datums or default_datums_path()) as f:
        return report(yaml.safe_load(f))


if __name__ == "__main__":
    sys.exit(main())
