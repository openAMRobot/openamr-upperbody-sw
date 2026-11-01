#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""P25: did anything move after the material change? Compares the three A6 and three A7 reports of P25 with the three of P24 (the same
code, platform-sw dfd44a5, before the material change), check by check.

    compare_after_p24.py EVIDENCE_DIR  P24_DEV P24_DEV P24_DEV  P24_A7 x3   P25_DEV x3  P25_A7 x3   (report file names, 3 + 3 + 3 + 3)

For each check: the result in every run; then, over the flattened `details`, the leaf values (numbers and strings) that differ between the
two groups, leaving out the volatile ones (durations, timestamps, pids, paths, hashes of a different commit). Nothing here is a claim about
the model beyond what those reports record."""
import json, os, re, sys

d = sys.argv[1]
names = sys.argv[2:]
groups = {"A6 P24": names[0:3], "A7 P24": names[3:6], "A6 P25": names[6:9], "A7 P25": names[9:12]}
VOLATILE = re.compile(r"(second|time|stamp|pid|_s$|duration|elapsed|path|file|dir|log|hash|sha|commit|date|session|domain|partition|port|rate|fps|seed|wall|real_|sim_time)", re.I)


def load(n):
    return json.load(open(os.path.join(d, n)))


def flat(x, prefix=""):
    if isinstance(x, dict):
        for k, v in x.items():
            yield from flat(v, f"{prefix}.{k}" if prefix else k)
    elif isinstance(x, list):
        if len(x) <= 40 and all(not isinstance(i, (dict, list)) for i in x):
            yield prefix, json.dumps(x)
        else:
            yield prefix + "[len]", len(x)
    else:
        yield prefix, x


def results(rep):
    return {c["id"]: c["result"] for c in rep["checks"]}


for kind in ("A6", "A7"):
    old = [load(n) for n in groups[f"{kind} P24"]]
    new = [load(n) for n in groups[f"{kind} P25"]]
    print(f"=== {kind}: P24 {[r['git_hash'][:7] for r in old]}  ->  P25 {[r['git_hash'][:7] for r in new]}")
    ids = sorted({i for r in old + new for i in results(r)})
    for i in ids:
        a = [results(r).get(i, "-") for r in old]
        b = [results(r).get(i, "-") for r in new]
        flag = "" if set(a) == set(b) else "   <-- DIFFERENT"
        print(f"  {i:4} P24 {','.join(a):22} P25 {','.join(b)}{flag}")
    for key in ("upstream", "datums_revision", "provenance"):
        sa = {json.dumps(r.get(key), sort_keys=True) for r in old}
        sb = {json.dumps(r.get(key), sort_keys=True) for r in new}
        print(f"  {key}: same in all six runs" if sa == sb and len(sa) == 1 else f"  {key}: DIFFERENT {len(sa)} vs {len(sb)} distinct values")
    ca = {json.dumps(r["config_ids"], sort_keys=True) for r in old}
    cb = {json.dumps(r["config_ids"], sort_keys=True) for r in new}
    print(f"  config_ids: {len(ca)} distinct set(s) in P24, {len(cb)} in P25; equal sets: {ca == cb} (expected False: the URDF text is an input of every id)")
    moved = 0
    for i in ids:
        oa, ob = [dict(flat(c["details"])) for r in old for c in r["checks"] if c["id"] == i], [dict(flat(c["details"])) for r in new for c in r["checks"] if c["id"] == i]
        keys = sorted({k for x in oa + ob for k in x if not VOLATILE.search(k)})
        for k in keys:
            va = {str(x.get(k, "<absent>")) for x in oa}
            vb = {str(x.get(k, "<absent>")) for x in ob}
            if not (vb <= va or va == vb):
                moved += 1
                print(f"  {i}.{k}: P24 {sorted(va)[:4]}  ->  P25 {sorted(vb)[:4]}")
    print(f"  details leaves whose P25 values fall outside the P24 values: {moved}\n")
