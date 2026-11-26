#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Counts the test cases of the Build 1 scene from JUnit XML and lists the scenario logs.

    count_build1_tests.py RESULTS_DIR [--logs DIR] [--summary FILE] [--label TEXT]

Test cases, not CTest entries: passed, failed, BLOCKED (a skip whose text contains BLOCKED) and skipped.
Exit 0: at least one passed and nothing failed, was BLOCKED or was skipped. Exit 1: a failure, or nothing ran.
Exit 3: nothing failed and at least one case is BLOCKED or skipped. BLOCKED and SKIP are not PASS.
XFAIL (a skip whose text starts with "XFAIL (#<issue>)"): an approved expected failure (scenarios.PARK_XFAIL), counted
apart and never as passed; it does not change the exit status.
"""
import argparse
import glob
import json
import os
import re
import sys
import xml.etree.ElementTree as ET


def classify(case):
    for tag in ("failure", "error"):
        if case.find(tag) is not None:
            return "failed", (case.find(tag).get("message") or case.find(tag).text or "")[:300]
    skipped = case.find("skipped")
    if skipped is not None:
        text = " ".join(filter(None, [skipped.get("message"), skipped.text]))
        if "BLOCKED" in text:
            return "blocked", text[:300]
        if re.match(r"XFAIL \(#\d+\)", text.strip()):
            return "xfail", text[:300]
        return "skipped", text[:300]
    return "passed", ""


def count(results_dir):
    rows = []
    for path in sorted(glob.glob(os.path.join(results_dir, "**", "*.xml"), recursive=True)):
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError as error:
            rows.append((os.path.basename(path), "unreadable", "failed", str(error)))
            continue
        for case in root.iter("testcase"):
            state, text = classify(case)
            rows.append((case.get("classname") or os.path.basename(path), case.get("name"), state, text))
    return rows


def scenario_logs(directory):
    logs = []
    for path in sorted(glob.glob(os.path.join(directory, "**", "*.json"), recursive=True)):
        try:
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, ValueError):
            continue
        if isinstance(data, dict) and "scenario" in data:
            logs.append(data)
    return logs


def report(rows, logs, label):
    totals = {state: sum(1 for row in rows if row[2] == state) for state in ("passed", "failed", "blocked", "skipped", "xfail")}
    lines = [f"### {label}: test cases", "",
             f"passed {totals['passed']}, failed {totals['failed']}, BLOCKED {totals['blocked']}, skipped {totals['skipped']}, "
             f"XFAIL {totals['xfail']}, total {len(rows)}. Simulation evidence only. BLOCKED, skipped and XFAIL are not "
             "passed; XFAIL is an approved expected failure with its issue.", ""]
    for classname, name, state, text in rows:
        if state != "passed":
            lines.append(f"- {state.upper()}: {classname} {name}: {text}")
    for data in logs:
        run = data.get("run", {})
        lines += ["", f"#### {data['scenario']}: {data.get('result')}", "",
                  f"Profile {run.get('profile')}, step {run.get('physics_step')}, engine {run.get('physics_engine')}, "
                  f"model sha256 {str(run.get('model_sha256'))[:12]}. {data.get('reason') or ''}", "",
                  "| Quantity | Value | Unit | Allowed or reference | Note |", "|---|---|---|---|---|"]
        for q in data.get("quantities", []):
            if q.get("kind") == "observed_today":
                limit = "" if q.get("expected_reference") is None else (
                    f"reference expects {q['expected_reference']}: {'matches' if q.get('matches_reference') else 'DIFFERS'}")
                name = f"{q['name']} (observed today)"
            else:
                limit = "" if q.get("ok") is None else f"{q.get('low')} to {q.get('high')}: {'ok' if q['ok'] else 'OUTSIDE'}"
                name = q["name"]
            lines.append(f"| {name} | {q.get('value')} | {q.get('unit', '')} | {limit} | {q.get('note', '')} |")
    return totals, "\n".join(lines) + "\n"


def exit_status(totals):
    if totals["failed"] or totals["passed"] == 0:
        return 1
    if totals["blocked"] or totals["skipped"]:
        return 3
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("results_dir")
    parser.add_argument("--logs", default=None)
    parser.add_argument("--summary", default=None)
    parser.add_argument("--label", default="Build 1 scene")
    args = parser.parse_args(argv)
    totals, text = report(count(args.results_dir), scenario_logs(args.logs) if args.logs else [], args.label)
    print(text)
    if args.summary:
        with open(args.summary, "a", encoding="utf-8") as f:
            f.write(text)
    return exit_status(totals)


if __name__ == "__main__":
    sys.exit(main())
