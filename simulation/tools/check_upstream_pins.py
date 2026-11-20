#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Report whether the pinned simulation packages are still served by PyPI, before a pull request finds out.

Checks simulation/requirements-xacro.txt (the xacro wheel, by its SHA-256) and the exact pins of simulation/requirements.txt
(by version). A pin fails when PyPI no longer has that version, the hashed file is gone or changed, or every file of the version
is yanked. Report only: nothing is changed, no issue is opened. Exit 0 when every pin is served, 1 otherwise, 2 on a network error.

  python3 simulation/tools/check_upstream_pins.py [--summary $GITHUB_STEP_SUMMARY]
"""
import argparse
import json
import pathlib
import re
import sys
import urllib.error
import urllib.request

SIM = pathlib.Path(__file__).resolve().parents[1]
FILES = (SIM / "requirements-xacro.txt", SIM / "requirements.txt")
LINE = re.compile(r"^([A-Za-z0-9_.\-]+)==([^\s;#]+)(?:\s+--hash=sha256:([0-9a-f]{64}))?\s*$")


class Unreachable(RuntimeError):
    """PyPI did not answer; nothing can be said about the pins."""


def parse(text):
    pins = []
    for raw in text.splitlines():
        line = raw.split("#")[0].strip()
        if not line:
            continue
        m = LINE.match(line)
        if not m:
            raise ValueError(f"not an exact pin: {raw!r}")
        pins.append({"name": m[1], "version": m[2], "sha256": m[3]})
    return pins


def fetch_pypi(name, version):
    """The PyPI JSON record of one release, or None when PyPI answers 404."""
    try:
        with urllib.request.urlopen(f"https://pypi.org/pypi/{name}/{version}/json", timeout=60) as r:
            return json.load(r)
    except urllib.error.HTTPError as e:
        if e.code == 404:
            return None
        raise Unreachable(f"{name} {version}: HTTP {e.code}") from e
    except (urllib.error.URLError, TimeoutError) as e:
        raise Unreachable(f"{name} {version}: {e}") from e


def check(pin, record):
    """(ok, text) for one pin and its PyPI record."""
    label = f"{pin['name']}=={pin['version']}"
    if record is None:
        return False, f"{label}: not served by PyPI (404)"
    files = record.get("urls") or []
    if pin["sha256"]:
        hit = [f for f in files if f.get("digests", {}).get("sha256") == pin["sha256"]]
        if not hit:
            return False, f"{label}: no file with the pinned sha256 {pin['sha256'][:12]} (served: {[f.get('filename') for f in files]})"
        if hit[0].get("yanked"):
            return False, f"{label}: {hit[0]['filename']} is yanked ({hit[0].get('yanked_reason') or 'no reason given'})"
        return True, f"{label}: {hit[0]['filename']} sha256 {pin['sha256'][:12]} served, not yanked"
    if not files:
        return False, f"{label}: the release has no files"
    if all(f.get("yanked") for f in files):
        return False, f"{label}: every file is yanked"
    return True, f"{label}: served ({len(files)} files)"


def main(argv=None, fetch=fetch_pypi, files=FILES):
    parser = argparse.ArgumentParser()
    parser.add_argument("--summary")
    opts = parser.parse_args(argv)
    lines, failed = [], 0
    try:
        for path in files:
            for pin in parse(pathlib.Path(path).read_text()):
                ok, text = check(pin, fetch(pin["name"], pin["version"]))
                failed += not ok
                lines.append(f"- {'served' if ok else 'NOT SERVED'}: {text} ({pathlib.Path(path).name})")
    except Unreachable as e:
        lines.append(f"- BLOCKED: PyPI not reachable ({e}); no verdict")
        failed = -1
    verdict = "all pins served" if failed == 0 else ("BLOCKED" if failed < 0 else f"{failed} pin(s) not served")
    report = "\n".join(["### Upstream pins (PyPI)", "", f"Verdict: {verdict}. Report only, nothing changed.", "", *lines, ""])
    print(report)
    if opts.summary:
        with open(opts.summary, "a") as out:
            out.write(report + "\n")
    return 2 if failed < 0 else (1 if failed else 0)


if __name__ == "__main__":
    sys.exit(main())
