#!/usr/bin/env python3
"""Check the frozen expansion set from the files alone: no ROS, no xacro, no workspace. Python 3.8+ standard library only.

    python3 verify_config_ids.py            # from this directory or any other

For every entry of index.json it recomputes the configuration id the way the project defines it (ARCHITECTURE.md s7;
scripts/print_config_id.py in the description package):

    id = "sha256:" + sha256(S).hexdigest()
    S  = UTF-8 of the JSON object {canonical_urdf, mast_id, arm_package, hardware_profile, datums_revision,
         openarm_description_commit} with keys sorted, separators "," and ":", ASCII-escaped
    canonical_urdf = XML C14N 2.0 of the file, comments dropped, whitespace-only text stripped

and compares it with the id in index.json and with the id in the file's own header comment. Exit status 0 only if every
file reproduces its id.
"""
import hashlib
import json
import os
import re
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
HEADER_ID = re.compile(r"config_id (sha256:[0-9a-f]{64})")


def config_id(text, entry, revision, upstream_commit):
    members = {"canonical_urdf": ET.canonicalize(text, strip_text=True), "mast_id": entry["mast_id"],
               "arm_package": entry["arm_package"], "hardware_profile": entry["hardware_profile"],
               "datums_revision": revision, "openarm_description_commit": upstream_commit}
    payload = json.dumps(members, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    return "sha256:" + hashlib.sha256(payload).hexdigest()


def main():
    with open(os.path.join(HERE, "index.json")) as f:
        index = json.load(f)
    bad = 0
    groups = [("expansions", index["entries"]), ("a7_variant", index.get("a7_variant", {}).get("entries", []))]
    for folder, entries in groups:
        source = index if folder == "expansions" else index["a7_variant"]
        for entry in entries:
            path = os.path.join(HERE, folder, entry["file"])
            with open(path, encoding="utf-8") as f:
                text = f.read()
            got = config_id(text, entry, source["datums_revision"], source["openarm_description_commit"])
            in_header = HEADER_ID.search(text[:2000])
            ok = got == entry["config_id"] and in_header is not None and in_header.group(1) == got
            bad += not ok
            print(f"{'OK  ' if ok else 'FAIL'} {folder}/{entry['file']}  {got}")
    print(f"{'all ids reproduce' if not bad else str(bad) + ' file(s) do not reproduce'}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
