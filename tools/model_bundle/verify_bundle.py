#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Check a model bundle against its own lock: every referenced mesh is listed, every listed file is there and its sha256
matches, and nothing is in the bundle that the lock does not list.

    verify_bundle.py <bundle directory or .zip>

Exit 0 if the bundle verifies, 1 with one line per problem otherwise. Python 3 only; no ROS needed.
"""
import hashlib
import json
import os
import re
import sys
import tempfile
import zipfile

LOCK = "bundle_lock.json"
SUMS = "SHA256SUMS"
MESH_RE = re.compile(r'filename="([^"]+)"')
PLUGIN_FILENAME_RE = re.compile(r"^[A-Za-z0-9_\-]+$")   # gz plugin library names, e.g. gz-sim-imu-system
ABSOLUTE_RE = re.compile(r'(?:filename|uri)="(?:file://|/)|(?<![A-Za-z0-9:.])/(?:home|tmp|opt|root|Users|__w|github)/')


def _sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def verify_dir(root):
    problems = []
    lock_path = os.path.join(root, LOCK)
    if not os.path.isfile(lock_path):
        return [f"{LOCK} missing"]
    with open(lock_path) as f:
        lock = json.load(f)
    listed = {rec["path"]: rec for rec in lock.get("files", [])}
    if not listed:
        problems.append("the lock lists no files")

    on_disk = set()
    for dirpath, _, filenames in os.walk(root):
        for fn in filenames:
            on_disk.add(os.path.relpath(os.path.join(dirpath, fn), root).replace(os.sep, "/"))
    for path in sorted(on_disk - set(listed) - {LOCK, SUMS}):
        problems.append(f"{path}: in the bundle, not in the lock")
    for path, rec in sorted(listed.items()):
        full = os.path.join(root, path)
        if not os.path.isfile(full):
            problems.append(f"{path}: in the lock, not in the bundle")
            continue
        if _sha256(full) != rec.get("sha256"):
            problems.append(f"{path}: sha256 differs from the lock")
        if rec.get("bytes") != os.path.getsize(full):
            problems.append(f"{path}: size differs from the lock")
        if rec.get("kind") == "mesh" and not rec.get("licence"):
            problems.append(f"{path}: mesh without a licence")

    configurations = lock.get("configurations", [])
    if not configurations:
        problems.append("the lock lists no configurations")
    referenced_all = set()
    for conf in configurations:
        urdf = os.path.join(root, conf["urdf"])
        if not os.path.isfile(urdf):
            problems.append(f"{conf['id']}: {conf['urdf']} missing")
            continue
        with open(urdf) as f:
            text = f.read()
        if ABSOLUTE_RE.search(text):
            problems.append(f"{conf['urdf']}: absolute file path in the model")
        referenced = set()
        for uri in MESH_RE.findall(text):
            if PLUGIN_FILENAME_RE.match(uri):
                continue
            if not uri.startswith("package://"):
                problems.append(f"{conf['urdf']}: mesh URI {uri} is not package://")
                continue
            referenced.add("share/" + uri[len("package://"):])
        referenced_all |= referenced
        for path in sorted(referenced):
            rec = listed.get(path)
            if rec is None:
                problems.append(f"{conf['urdf']}: references {path}, which the lock does not list")
            elif rec.get("kind") != "mesh":
                problems.append(f"{conf['urdf']}: references {path}, listed as {rec.get('kind')}, not mesh")
        if sorted(referenced) != sorted(conf.get("meshes", [])):
            problems.append(f"{conf['id']}: the lock's mesh list differs from the meshes the URDF references")
    for path, rec in sorted(listed.items()):
        if rec.get("kind") == "mesh" and path not in referenced_all:
            problems.append(f"{path}: listed as a mesh, referenced by no URDF")

    sums_path = os.path.join(root, SUMS)
    if not os.path.isfile(sums_path):
        problems.append(f"{SUMS} missing")
    else:
        expected = {p: r["sha256"] for p, r in listed.items()}
        expected[LOCK] = _sha256(lock_path)
        seen = {}
        with open(sums_path) as f:
            for line in f:
                if line.strip():
                    digest, _, path = line.rstrip("\n").partition("  ")
                    seen[path] = digest
        if seen != expected:
            problems.append(f"{SUMS} does not match the lock and the bundle")
    return problems


def verify(path):
    if zipfile.is_zipfile(path):
        with tempfile.TemporaryDirectory() as tmp:
            with zipfile.ZipFile(path) as z:
                names = z.namelist()
                tops = {n.split("/", 1)[0] for n in names}
                if len(tops) != 1 or any(n.startswith("/") or ".." in n.split("/") for n in names):
                    return ["the zip must hold exactly one top-level directory and only relative paths"]
                z.extractall(tmp)
            return verify_dir(os.path.join(tmp, tops.pop()))
    return verify_dir(path)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print(__doc__, file=sys.stderr)
        return 2
    problems = verify(argv[0])
    for p in problems:
        print(f"FAIL  {p}")
    if not problems:
        print(f"PASS  {argv[0]}: every referenced mesh listed, every file present with its sha256")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
