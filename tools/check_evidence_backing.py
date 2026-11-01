#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Evidence-backing check (P22 task D, a prototype for this repository only).

Every count and rate that our PR body or a generated report states must either resolve to a file under evidence/ or be printed as UNBACKED.

    check_evidence_backing.py [--doc PATH ...] [--report PATH ...] [--ledger PATH] [--json OUT] [--verbose]
    check_evidence_backing.py --skeleton           # print ledger entries for the claims that nothing resolves yet
    check_evidence_backing.py --canary             # plant unbacked numbers in a temporary tree and show they are caught

Why it exists. A report to the architecture chat said `parameter_bridge` exits -11 in "about 1 in 7" G02 runs. The repository has no record of
any such exit before P19; the figure had nothing behind it. This is the check that would have printed it as UNBACKED.

What it reads. Documents (default `docs/pr/PR14_BODY.md`) and the prose strings of generated reports (default the newest committed A6 `*_dev.json`
and A7 `*_a7.json` under `evidence/reports/`; add a fresh one with --report). Fenced code blocks and HTML comments are skipped. It looks for
the claims that are counts and rates, not prose:

  verdict counts   `25 PASS / 3 WARN / 0 FAIL`, with or without NOT_RUN, and the bare `25 / 3 / 0` of a results table
  rates            `2 of 36`, `1 in 7`, `about 1 in 13`; digits, and a number word only when a count noun follows (`two of the eight runs`)
  counts           `826 tests`, `95 pairs`, `36 launches`, `0 excursions` (a number and one of a fixed list of count nouns)
  percentages      `20%`

What a claim can resolve to (ledger: `docs/pr/claim_backing.yaml`; a claim is matched by its normalised text, in the document it was found in):

  BACKED      a verdict count that a committed report has, found automatically; or a `json_count` entry whose count over the committed reports
              equals the claim (a rate: numerator and denominator both)
  RECORDED    a `record` entry: the named file under evidence/ contains the stated text. The file is a written record, not data derived from
              the reports, and the status says so; it is weaker than BACKED on purpose
  BACKED (launch_exits)
              a rate of a process exit code derived from the `process_exits` an A7 report carries since P24 (the exit code of every process of every
              recorded launch): "1 of 12 launches with a parameter_bridge ended in -11". Only recorded launches count; reports before P24 contribute none
  OBSERVED-NOT-RECORDED
              an `observed_not_recorded` entry: someone saw it (a console line, say) and nothing was recorded. Printed as such, does not fail, and
              only while the text on the claim's line says OBSERVED-NOT-RECORDED, so a reader of the document sees the status too
  UNBACKED    a `unbacked` entry, with a reason: acknowledged and still printed as UNBACKED, so the list of what has nothing behind it is a
              list a reviewer can read. A claim with no entry at all is UNBACKED and fails
  MISMATCH    an entry whose backing does not say what the claim says (the file is missing, the text is not in it, the count differs): fails
  UNUSED      a ledger entry that matches no claim any more (the text was reworded or removed): fails, so the ledger cannot rot

Not covered, said so here and not silently: numbers inside prose that are not counts of the nouns above (sizes, durations, versions); counts
written as words outside a rate; numbers in tables that carry no label. Backed means "a file under evidence/ shows it", not "the number is
right": a record written by us is only as good as we were.

Exit status: 0 every claim BACKED, RECORDED or acknowledged UNBACKED; 1 at least one unacknowledged UNBACKED, MISMATCH or UNUSED; 2 an input
cannot be read.
"""
import argparse
import glob
import json
import os
import re
import sys
import tempfile

try:
    import yaml
except ImportError:                                     # pragma: no cover
    sys.exit("check_evidence_backing: PyYAML is required (python3-yaml)")

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
DEFAULT_LEDGER = os.path.join(REPO, "docs", "pr", "claim_backing.yaml")
DEFAULT_DOCS = ["docs/pr/PR14_BODY.md"]
REPORT_GLOBS = ["evidence/reports/*_dev.json", "evidence/reports/*_a7.json"]

WORDS = "one two three four five six seven eight nine ten eleven twelve".split()
WORD = "(?:%s)" % "|".join(WORDS)
NOUNS = ("tests?|passed|failed|skipped|launches|runs|pairs|orphans|activations|excursions|joints|poses|samples|reports|errors|commits|files|"
         "dependencies|checks|frames")
INT = r"\d+(?:,\d{3})*"                 # an integer, thousands commas only
NUM = INT
PATTERNS = [
    ("verdicts", re.compile(r"\b\d+\s+PASS\b[^|\n]{0,60}?\b\d+\s+WARN\b[^|\n]{0,60}?\b\d+\s+FAIL\b(?:[^|\n]{0,40}?\b\d+\s+NOT_RUN\b)?")),
    ("bare_triple", re.compile(r"(?<![\w.,])%s / %s / %s(?: / %s)?(?!\w)(?!,\d)(?!\.\d)" % ((INT,) * 4))),
    ("rate_of", re.compile(r"(?<![\d.,])%s of (?:about |the )?%s\b(?!\.\d)" % (NUM, NUM))),
    ("rate_in", re.compile(r"(?<![\d.,])(?:about |roughly )?%s in (?:about )?%s\b(?!\.\d)" % (NUM, NUM))),
    ("rate_words", re.compile(r"\b(?:%s of (?:the )?(?:%s|%s)|%s in (?:about )?(?:%s|%s)) (?:[A-Za-z0-9]+ ){0,2}?(?:runs|launches|reports|tests|pairs|samples)\b"
                              % (WORD, WORD, NUM, WORD, WORD, NUM), re.I)),
    ("count", re.compile(r"\b%s\s+(?:%s)\b" % (NUM, NOUNS))),
    ("percent", re.compile(r"\b\d+(?:\.\d+)?\s*(?:%|per cent|percent)")),
]
CHECK_COUNT_FIELDS = ("PASS", "WARN", "FAIL", "NOT_RUN")
OBSERVED_LABEL = "observed-not-recorded"        # normalised: the text must carry OBSERVED-NOT-RECORDED next to such a number
ROW_DOCS = ["evidence/reports/README.md"]       # tables whose rows name report files next to a verdict count: only those counts are read
REPORT_NAME = re.compile(r"(?:(\d{8})T(\d{6})Z|\.\.\.T(\d{6})Z)_(dev|a7)\.json")


# --- claims ---------------------------------------------------------------------------------------------------------------------

def normalise(text):
    return re.sub(r"\s+", " ", text.replace("**", "").replace("`", "")).strip().lower()


def find_claims(text):
    """[(claim_text, kind)] for one string, non-overlapping, earlier patterns first."""
    taken, found = [], []
    for kind, regex in PATTERNS:
        for m in regex.finditer(text):
            if any(m.start() < end and start < m.end() for start, end in taken):
                continue
            taken.append((m.start(), m.end()))
            found.append((m.group(0), kind))
    return found


def claims_in_markdown(path, text, rows_only=False):
    claims, fenced, in_comment = [], False, False
    for number, raw in enumerate(text.split("\n"), 1):
        if raw.lstrip().startswith("```"):
            fenced = not fenced
            continue
        if fenced:
            continue
        line = raw
        if in_comment:
            if "-->" in line:
                line, in_comment = line.split("-->", 1)[1], False
            else:
                continue
        while "<!--" in line:
            before, rest = line.split("<!--", 1)
            if "-->" in rest:
                line = before + rest.split("-->", 1)[1]
            else:
                line, in_comment = before, True
        clean = line.replace("**", "").replace("`", "")
        row_files = report_names_in_row(clean) if clean.lstrip().startswith("|") else []
        for claim, kind in find_claims(clean):
            record = {"doc": path, "where": f"{path}:{number}", "text": normalise(claim), "shown": claim.strip(), "kind": kind,
                      "line": normalise(clean)}
            if row_files and kind == "verdicts":
                record["row_files"] = row_files
            if rows_only and not (row_files and kind == "verdicts"):
                continue
            claims.append(record)
    return claims


def report_names_in_row(row):
    """Report file names a table row names; `...T121946Z_a7.json` borrows the date of the first full name in the row."""
    names, date = [], None
    for m in REPORT_NAME.finditer(row):
        if m.group(1):
            date = m.group(1)
            names.append(f"{m.group(1)}T{m.group(2)}Z_{m.group(4)}.json")
        elif date:
            names.append(f"{date}T{m.group(3)}Z_{m.group(4)}.json")
    return names


def json_strings(obj, path=""):
    if isinstance(obj, dict):
        for key, value in obj.items():
            yield from json_strings(value, f"{path}/{key}")
    elif isinstance(obj, list):
        for index, value in enumerate(obj):
            yield from json_strings(value, f"{path}/{index}")
    elif isinstance(obj, str):
        yield path, obj


def claims_in_report(path, data):
    """Claims in the prose strings of one generated report. The ledger key is `report:<profile>` (a7, dev), not the file name: the
    generator writes the same sentences into every run, and a new report must not need a new ledger."""
    claims = []
    profile = data.get("profile") if isinstance(data, dict) else None
    profile = profile or re.search(r"_(dev|a7)\.json$", path or "") and re.search(r"_(dev|a7)\.json$", path).group(1) or "unknown"
    key = f"report:{'a7' if profile == 'a7' else 'dev' if profile in ('dev', 'gazebo') else profile}"
    for where, string in json_strings(data):
        if len(string) < 20 or string.startswith("sha256:"):
            continue
        for claim, kind in find_claims(string.replace("**", "").replace("`", "")):
            claims.append({"doc": key, "where": f"{path}#{where}", "text": normalise(claim), "shown": claim.strip(), "kind": kind,
                           "line": normalise(string)})
    return claims


# --- backing --------------------------------------------------------------------------------------------------------------------

def numbers(text):
    out = []
    for token in re.findall(r"\d[\d,]*(?:\.\d+)?|\b(?:%s)\b" % "|".join(WORDS), text, re.I):
        low = token.lower()
        out.append(WORDS.index(low) + 1 if low in WORDS else float(token.replace(",", "")))
    return out


def load_reports(repo_root, patterns=None):
    """{relative path: parsed report} of every committed report matching the globs under evidence/reports/."""
    reports = {}
    for pattern in patterns or ["evidence/reports/*.json"]:
        for path in sorted(glob.glob(os.path.join(repo_root, pattern))):
            try:
                with open(path, encoding="utf-8") as f:
                    reports[os.path.relpath(path, repo_root).replace(os.sep, "/")] = json.load(f)
            except (OSError, ValueError):
                continue
    return reports


def verdict_counts(report):
    counts = {k: 0 for k in CHECK_COUNT_FIELDS}
    for check in report.get("checks") or []:
        if check.get("result") in counts:
            counts[check["result"]] += 1
    return counts


def dotted(data, path):
    """data[a][b]... for 'a.b'; a segment 'checks[M03]' picks the element of the list `checks` whose `id` is M03."""
    for part in path.split("."):
        m = re.fullmatch(r"([\w-]+)\[([\w-]+)\]", part)
        if m:
            data = data.get(m.group(1)) if isinstance(data, dict) else None
            data = next((e for e in data if isinstance(e, dict) and e.get("id") == m.group(2)), None) if isinstance(data, list) else None
        elif isinstance(data, dict) and part in data:
            data = data[part]
        else:
            return None
        if data is None:
            return None
    return data


def count_reports(repo_root, spec):
    """How many committed reports match `spec` ({glob or globs, where: [...]}) -> (count, [names])."""
    names = []
    globs = spec.get("globs") or [spec["glob"]]
    for name, report in load_reports(repo_root, globs).items():
        ok = True
        for condition in spec.get("where") or []:
            if "path" in condition and "startswith" in condition:
                value = dotted(report, condition["path"])
                ok &= isinstance(value, str) and value.startswith(condition["startswith"])
            elif "path" in condition:
                ok &= dotted(report, condition["path"]) == condition.get("equals")
            elif "check" in condition:
                ok &= any(c.get("id") == condition["check"] and c.get("result") == condition["result"] for c in report.get("checks") or [])
            elif "any_check_result" in condition:
                ok &= any(c.get("result") == condition["any_check_result"] for c in report.get("checks") or [])
            elif "no_check_result" in condition:
                ok &= not any(c.get("result") == condition["no_check_result"] for c in report.get("checks") or [])
            elif "verdicts" in condition:
                ok &= verdict_counts(report) == {**{k: 0 for k in CHECK_COUNT_FIELDS}, **condition["verdicts"]}
            else:
                ok = False
        if ok:
            names.append(name)
    return len(names), names


def count_launches(repo_root, spec):
    """(with the process, with the process ending in the code, [report names]) over the `process_exits` of the committed reports matching the
    globs (P24). `process_exits` is the A7 report's list of launches, each with the exit code of each of its processes; a launch counts when it has a
    process whose name starts with `spec["process"]`. Reports that have no `process_exits` (everything before P24) contribute nothing, which is the
    point: only launches whose exit codes were recorded can back a rate."""
    names, total, hit = [], 0, 0
    for name, report in load_reports(repo_root, spec.get("globs") or [spec["glob"]]).items():
        launches = report.get("process_exits")
        if not isinstance(launches, list):
            continue
        names.append(name)
        for launch in launches:
            procs = [p for p in (launch.get("processes") or []) if str(p.get("process", "")).startswith(spec["process"])]
            if procs:
                total += 1
                hit += any(p.get("exit_code") == spec["exit_code"] for p in procs)
    return hit, total, names


def under_evidence(repo_root, relative):
    """Absolute path of `relative` if it names a file under evidence/ inside the repository, else None."""
    full = os.path.normpath(os.path.join(repo_root, relative))
    root = os.path.normpath(os.path.join(repo_root, "evidence")) + os.sep
    return full if full.startswith(root) and os.path.isfile(full) else None


def resolve(claim, entry, repo_root, reports):
    """(status, detail, evidence_files) for one claim and its ledger entry (or None)."""
    if claim.get("row_files") and entry is None:
        wanted = {label: int(n) for n, label in re.findall(r"(\d+)\s+(PASS|WARN|FAIL|NOT_RUN)\b", claim["text"].upper())}
        problems, found = [], []
        for name in claim["row_files"]:
            report = reports.get("evidence/reports/" + name)
            if report is None:
                problems.append(f"{name} is not a committed report")
                continue
            have = verdict_counts(report)
            found.append("evidence/reports/" + name)
            if any(have[label] != value for label, value in wanted.items()):
                problems.append(f"{name} has {have['PASS']} PASS / {have['WARN']} WARN / {have['FAIL']} FAIL / {have['NOT_RUN']} NOT_RUN")
        if problems:
            return "MISMATCH", "the report(s) named in the same row do not have these counts: " + "; ".join(problems), found
        return "BACKED", "every report named in the same row has exactly these verdict counts", found[:3]
    if claim["kind"] == "bare_triple" and entry is None:
        wanted = [int(n) for n in numbers(claim["text"])]
        if len(wanted) == 3:
            matches = [name for name, report in reports.items()
                       if [verdict_counts(report)[k] for k in ("PASS", "WARN", "FAIL")] == wanted]
            if matches:
                return "BACKED", f"{len(matches)} committed report(s) have PASS / WARN / FAIL = {wanted[0]} / {wanted[1]} / {wanted[2]}", matches[:3]
    if claim["kind"] == "verdicts" and entry is None:
        wanted = {label: int(n) for n, label in re.findall(r"(\d+)\s+(PASS|WARN|FAIL|NOT_RUN)\b", claim["text"].upper())}
        matches = [name for name, report in reports.items()
                   if all(verdict_counts(report)[label] == value for label, value in wanted.items())]
        if matches:
            return "BACKED", f"{len(matches)} committed report(s) have exactly these verdict counts", matches[:3]
    if entry is None:
        return "UNBACKED", "no committed report shows it and the ledger has no entry", []
    kind = entry.get("kind")
    if kind == "unbacked":
        return "UNBACKED", "acknowledged: " + str(entry.get("reason", "")).strip(), []
    if kind == "observed_not_recorded":
        if OBSERVED_LABEL not in claim.get("line", ""):
            return "MISMATCH", f"this claim is an observation nothing recorded, and the text around it must say {OBSERVED_LABEL.upper()}", []
        return "OBSERVED-NOT-RECORDED", "seen by a person (" + str(entry.get("observed_by", "console")).strip() + "); no file under evidence/ carries it as data", []
    if kind == "record":
        path = under_evidence(repo_root, entry.get("file", ""))
        if path is None:
            return "MISMATCH", f"{entry.get('file')} is not a file under evidence/", []
        with open(path, encoding="utf-8", errors="replace") as f:
            body = f.read()
        needle = entry.get("contains") or claim["shown"]
        if normalise(needle) not in normalise(body):
            return "MISMATCH", f"{entry['file']} does not contain {needle!r}", []
        return "RECORDED", f"written down in {entry['file']} (a record, not data derived from the reports)", [entry["file"]]
    if kind == "json_count":
        want = [int(n) for n in numbers(claim["text"])]
        if "numerator" in entry:
            have_n, names_n = count_reports(repo_root, entry["numerator"])
            have_d, names_d = count_reports(repo_root, entry["denominator"])
            if [have_n, have_d] != want[:2]:
                return "MISMATCH", f"the committed reports give {have_n} of {have_d}; the claim says {want[0]} of {want[1]}", []
            return "BACKED", f"{have_n} of {have_d} committed reports", sorted(set(names_n) | set(names_d))[:3]
        have, names = count_reports(repo_root, entry)
        expected = entry.get("expect", want[0] if want else None)
        if have != expected or (want and not entry.get("count_not_in_claim") and have != want[0]):
            return "MISMATCH", f"the committed reports give {have}; the claim says {want[0] if want else '?'}", []
        return "BACKED", f"{have} committed report(s)", names[:3]
    if kind == "launch_exits":
        hit, total, names = count_launches(repo_root, entry)
        want = [int(n) for n in numbers(claim["text"])]
        if want[:2] != [hit, total]:
            return "MISMATCH", f"the recorded exit codes give {hit} of {total} launches with {entry['process']} ending in {entry['exit_code']}; the claim says {' of '.join(map(str, want[:2]))}", []
        return "BACKED", f"{hit} of {total} recorded launches (process_exits of {len(names)} reports): {entry['process']} exit code {entry['exit_code']}", names[:3]
    if kind == "json_all":
        have, names = count_reports(repo_root, entry)
        reports_all = load_reports(repo_root, entry.get("globs") or [entry["glob"]])
        differing = [n for n in names if dotted(reports_all[n], entry["path"]) != entry.get("equals")]
        if len(names) < entry.get("at_least", 1) or differing:
            return "MISMATCH", f"{len(names)} report(s) match; {len(differing)} of them do not have {entry['path']} == {entry.get('equals')!r}", []
        return "BACKED", f"{entry['path']} == {entry.get('equals')!r} in all {len(names)} committed reports", names[:3]
    if kind == "json_value":
        path = under_evidence(repo_root, entry.get("file", ""))
        if path is None:
            return "MISMATCH", f"{entry.get('file')} is not a file under evidence/", []
        try:
            with open(path, encoding="utf-8") as f:
                value = json.load(f)
        except (OSError, ValueError):
            return "MISMATCH", f"{entry['file']} is not JSON", []
        if entry.get("path"):
            value = dotted(value, entry["path"]) if isinstance(value, dict) else None
        measured = len(value) if entry.get("length") and value is not None else value
        want = [int(n) for n in numbers(claim["text"])]
        if measured is None or want[:1] != [measured]:
            return "MISMATCH", f"{entry['file']} gives {measured!r}; the claim says {want[0] if want else '?'}", []
        return "BACKED", f"{entry['file']} {entry.get('path', '')}{' (length)' if entry.get('length') else ''} = {measured}", [entry["file"]]
    return "MISMATCH", f"unknown ledger kind {kind!r}", []


def run(repo_root, ledger_path, docs, report_paths, extra_reports=()):
    try:
        with open(ledger_path, encoding="utf-8") as f:
            ledger = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise SystemExit_(2, f"cannot read the ledger {ledger_path}: {exc}")
    entries = ledger.get("claims") or []
    by_key, by_key_near = {}, {}
    for entry in entries:
        key = (entry.get("doc"), normalise(str(entry.get("claim", ""))))
        if entry.get("near"):
            by_key_near.setdefault(key, []).append(entry)          # an entry for one place only: the claim's line must contain `near`
        else:
            by_key.setdefault(key, entry)
    claims = []
    for relative in list(docs) + [d for d in ROW_DOCS if d not in docs]:
        path = os.path.join(repo_root, relative)
        try:
            with open(path, encoding="utf-8") as f:
                claims.extend(claims_in_markdown(relative, f.read(), rows_only=relative in ROW_DOCS and relative not in docs))
        except OSError as exc:
            if relative in ROW_DOCS and relative not in docs:
                continue                                   # a repository without the index has no rows to check
            raise SystemExit_(2, f"cannot read {relative}: {exc}")
    for relative in list(report_paths) + list(extra_reports):
        path = relative if os.path.isabs(relative) else os.path.join(repo_root, relative)
        label = os.path.relpath(path, repo_root).replace(os.sep, "/") if path.startswith(repo_root) else path
        try:
            with open(path, encoding="utf-8") as f:
                claims.extend(claims_in_report(label, json.load(f)))
        except (OSError, ValueError) as exc:
            raise SystemExit_(2, f"cannot read the report {relative}: {exc}")
    reports = load_reports(repo_root)
    used, results = set(), []
    for claim in claims:
        key = (claim["doc"], claim["text"])
        entry = next((e for e in by_key_near.get(key, []) if normalise(e["near"]) in claim.get("line", "")), None)
        if entry is not None:
            used.add((key, normalise(entry["near"])))
        else:
            entry = by_key.get(key)
            if entry is not None:
                used.add(key)
        status, detail, files = resolve(claim, entry, repo_root, reports)
        if status == "UNBACKED" and entry is None:
            status = "UNBACKED!"                         # not acknowledged: the one that fails
        results.append({**claim, "status": status, "detail": detail, "evidence": files})
    scanned = set(docs) | {c["doc"] for c in claims}
    every = [(key, entry, key) for key, entry in by_key.items()] + [(key, entry, (key, normalise(entry["near"]))) for key, group in by_key_near.items() for entry in group]
    for key, entry, token in every:
        if token not in used and entry.get("doc") in scanned:
            results.append({"doc": key[0], "where": key[0], "text": key[1], "shown": key[1], "kind": "ledger", "status": "UNUSED",
                            "detail": "a ledger entry that matches no claim in the document" + (f" (near {entry['near']!r})" if entry.get("near") else ""), "evidence": []})
    failing = [r for r in results if r["status"] in ("UNBACKED!", "MISMATCH", "UNUSED")]
    counts = {}
    for r in results:
        counts[r["status"]] = counts.get(r["status"], 0) + 1
    return {"schema": "openamr.evidence_backing.v1", "status": "FAIL" if failing else "PASS", "counts": counts, "claims": results,
            "documents": list(docs), "reports": list(report_paths) + list(extra_reports)}


class SystemExit_(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def newest_reports(repo_root):
    out = []
    for pattern in REPORT_GLOBS:
        files = sorted(glob.glob(os.path.join(repo_root, pattern)))
        if files:
            out.append(os.path.relpath(files[-1], repo_root).replace(os.sep, "/"))
    return out


def format_report(report, verbose=False):
    lines = []
    for r in report["claims"]:
        if r["status"] in ("BACKED", "RECORDED") and not verbose:
            continue
        files = f"  [{', '.join(r['evidence'])}]" if r["evidence"] else ""
        label = "UNBACKED" if r["status"] == "UNBACKED!" else r["status"]
        bang = "  <-- not acknowledged" if r["status"] == "UNBACKED!" else ""
        lines.append(f"{label:<9} {r['where']}  {r['shown']!r}  {r['detail']}{files}{bang}")
    summary = ", ".join(f"{k.rstrip('!')}{' (unacknowledged)' if k.endswith('!') else ''} {v}" for k, v in sorted(report["counts"].items()))
    lines.append(f"{report['status']}: {len(report['claims'])} claims: {summary}")
    return "\n".join(lines)


def skeleton(report):
    seen, out = set(), []
    for r in report["claims"]:
        if r["status"] != "UNBACKED!" or (r["doc"], r["text"]) in seen:
            continue
        seen.add((r["doc"], r["text"]))
        out.append(f"  - {{doc: {r['doc']}, claim: {json.dumps(r['text'])}, kind: unbacked, reason: TODO}}   # {r['where']}")
    return "\n".join(out)


# --- canary ----------------------------------------------------------------------------------------------------------------------

def canary(repo_root, ledger_path):
    """Plant unbacked numbers in a temporary tree and show each is caught. Returns (ok, lines). Never writes to the tracked files."""
    lines, ok = [], True
    real = run(repo_root, ledger_path, DEFAULT_DOCS, newest_reports(repo_root))
    lines.append(f"baseline (the real documents and reports): {real['status']}   {real['counts']}")
    with tempfile.TemporaryDirectory(prefix="evidence_backing_canary_") as tmp:
        os.makedirs(os.path.join(tmp, "evidence", "reports"))
        os.makedirs(os.path.join(tmp, "docs", "pr"))
        report = {"checks": [{"id": "A", "result": "PASS"}, {"id": "B", "result": "PASS"}, {"id": "C", "result": "WARN"}],
                  "upstream": {"platform": "abc1234"}, "limitations": ["stands alone"]}
        with open(os.path.join(tmp, "evidence", "reports", "20260101T000000Z_a7.json"), "w") as f:
            json.dump(report, f)
        with open(os.path.join(tmp, "evidence", "reports", "README.md"), "w") as f:
            f.write("Three runs were made on the pin; one of them failed.\n")
        base_doc = "A7 gave 2 PASS / 1 WARN / 0 FAIL. It ran on the pin in 1 of 1 reports.\n"

        def attempt(label, doc_text, ledger_claims, expect, reports=(), docs=("docs/pr/BODY.md",)):
            with open(os.path.join(tmp, "docs", "pr", "BODY.md"), "w") as f:
                f.write(doc_text)
            ledger = os.path.join(tmp, "ledger.yaml")
            with open(ledger, "w") as f:
                yaml.safe_dump({"claims": ledger_claims}, f)
            result = run(tmp, ledger, list(docs), list(reports))
            statuses = {r["status"] for r in result["claims"]}
            caught = (expect in statuses and result["status"] == "FAIL") if expect else result["status"] == "PASS"
            return caught, sorted(statuses), result

        backed = [{"doc": "docs/pr/BODY.md", "claim": "1 of 1", "kind": "json_count",
                   "numerator": {"glob": "evidence/reports/*_a7.json", "where": [{"path": "upstream.platform", "equals": "abc1234"}]},
                   "denominator": {"glob": "evidence/reports/*_a7.json"}}]
        cases = [
            ("a clean document: every claim backed", base_doc, backed, None),
            ("a rate nothing backs, planted in the body (the 'about 1 in 7' shape)", base_doc + "The bridge dies in about 1 in 7 runs.\n", backed, "UNBACKED!"),
            ("a verdict count no report has", base_doc + "Then 9 PASS / 0 WARN / 0 FAIL.\n", backed, "UNBACKED!"),
            ("a percentage nothing backs", base_doc + "That is a 20% chance.\n", backed, "UNBACKED!"),
            ("a rate in words with a count noun", base_doc + "Two of the eight runs failed.\n", backed, "UNBACKED!"),
            ("a rate whose ledger entry counts something else", base_doc,
             [{**backed[0], "denominator": {"glob": "evidence/reports/*_a7.json", "where": [{"path": "upstream.platform", "equals": "nope"}]}}], "MISMATCH"),
            ("a record that is not in the file it names", base_doc + "There were 3 runs.\n",
             backed + [{"doc": "docs/pr/BODY.md", "claim": "3 runs", "kind": "record", "file": "evidence/reports/README.md", "contains": "four runs"}], "MISMATCH"),
            ("a record that points outside evidence/", base_doc + "There were 3 runs.\n",
             backed + [{"doc": "docs/pr/BODY.md", "claim": "3 runs", "kind": "record", "file": "docs/pr/BODY.md"}], "MISMATCH"),
            ("a ledger entry for a claim that is no longer in the document", base_doc,
             backed + [{"doc": "docs/pr/BODY.md", "claim": "7 pairs", "kind": "unbacked", "reason": "gone"}], "UNUSED"),
        ]
        for label, doc_text, ledger_claims, expect in cases:
            caught, statuses, _ = attempt(label, doc_text, ledger_claims, expect)
            ok &= caught
            lines.append(f"{'CAUGHT ' if caught and expect else 'OK     ' if caught else 'MISSED '} {label}: statuses {statuses}")
        # An acknowledged UNBACKED is printed and does not fail; the same number unacknowledged does.
        acked = backed + [{"doc": "docs/pr/BODY.md", "claim": "about 1 in 7", "kind": "unbacked", "reason": "quoted so it can be retracted"}]
        caught, statuses, result = attempt("acknowledged", base_doc + "The bridge dies in about 1 in 7 runs.\n", acked, None)
        printed = any(r["status"] == "UNBACKED" for r in result["claims"])
        ok &= caught and printed
        lines.append(f"{'OK     ' if caught and printed else 'WRONG  '} an acknowledged UNBACKED claim is still printed as UNBACKED and does not fail: statuses {statuses}")
        # A table row in the reports index whose verdict counts are not those of the report it names (the P17 "13 PASS" that was 12).
        with open(os.path.join(tmp, "evidence", "reports", "README.md"), "w") as f:
            f.write("| file | result |\n|---|---|\n| `20260101T000000Z_a7.json` | 3 PASS, 0 WARN, 0 FAIL |\n")
        caught, statuses, _ = attempt("row", base_doc, backed, "MISMATCH")
        ok &= caught
        lines.append(f"{'CAUGHT ' if caught else 'MISSED '} a report index row whose counts differ from the report it names (3 PASS against 2): statuses {statuses}")
        with open(os.path.join(tmp, "evidence", "reports", "README.md"), "w") as f:
            f.write("| file | result |\n|---|---|\n| `20260101T000000Z_a7.json` | 2 PASS, 1 WARN, 0 FAIL |\n")
        caught, statuses, _ = attempt("row", base_doc, backed, None)
        ok &= caught
        lines.append(f"{'OK     ' if caught else 'WRONG  '} the same row with the report's own counts is clean: statuses {statuses}")
        # A synthetic -11 (P24): the rate derives from the recorded exit codes, and from nothing else.
        def launch(check, code):
            return {"check": check, "sidecar": f"{check}_x.json",
                    "processes": [{"process": "parameter_bridge-4", "exit_code": code}, {"process": "gazebo-1", "exit_code": -2}]}
        syn = dict(report, process_exits=[launch("G01", 0), launch("G02", -11), launch("M02", 0), launch("M07", 0)])
        tree_root = os.path.join(tmp, "evidence", "reports", "20260103T000000Z_exits.json")
        with open(tree_root, "w") as f:
            json.dump(syn, f)
        spec = {"doc": "docs/pr/BODY.md", "claim": "1 of 4", "kind": "launch_exits", "glob": "evidence/reports/*_exits.json", "process": "parameter_bridge", "exit_code": -11}
        caught, statuses, result = attempt("derived", base_doc + "A bridge ended in -11 in 1 of 4 launches.\n", backed + [spec], None)
        derived = any(r["status"] == "BACKED" and "recorded launches" in r["detail"] for r in result["claims"])
        ok &= caught and derived
        lines.append(f"{'OK     ' if caught and derived else 'WRONG  '} a synthetic -11 in one of four recorded launches: '1 of 4' is derived from the recorded exit codes: statuses {statuses}")
        caught, statuses, _ = attempt("derived", base_doc + "A bridge ended in -11 in 2 of 4 launches.\n", backed + [{**spec, "claim": "2 of 4"}], "MISMATCH")
        ok &= caught
        lines.append(f"{'CAUGHT ' if caught else 'MISSED '} the same data does not back '2 of 4': statuses {statuses}")
        os.remove(tree_root)
        # An observation nothing recorded: allowed only while the text says so.
        obs = backed + [{"doc": "docs/pr/BODY.md", "claim": "2 of 9", "kind": "observed_not_recorded", "observed_by": "console"}]
        caught, statuses, result = attempt("observed", base_doc + "The exit was seen in 2 of 9 launches.\n", obs, "MISMATCH")
        ok &= caught
        lines.append(f"{'CAUGHT ' if caught else 'MISSED '} an observed-only rate whose text does not say OBSERVED-NOT-RECORDED: statuses {statuses}")
        caught, statuses, result = attempt("observed", base_doc + "The exit was seen (OBSERVED-NOT-RECORDED) in 2 of 9 launches.\n", obs, None)
        printed = any(r["status"] == "OBSERVED-NOT-RECORDED" for r in result["claims"])
        ok &= caught and printed
        lines.append(f"{'OK     ' if caught and printed else 'WRONG  '} the same rate labelled OBSERVED-NOT-RECORDED is printed with that status and does not fail: statuses {statuses}")
        # A planted number in a generated report's prose.
        planted = dict(report, limitations=["the bridge exits -11 in 3 of 5 launches"])
        with open(os.path.join(tmp, "evidence", "reports", "20260102T000000Z_dev.json"), "w") as f:
            json.dump(planted, f)
        caught, statuses, _ = attempt("report", base_doc, backed, "UNBACKED!", reports=["evidence/reports/20260102T000000Z_dev.json"])
        ok &= caught
        lines.append(f"{'CAUGHT ' if caught else 'MISSED '} a number planted in a generated report's prose: statuses {statuses}")
    lines.append(f"canary: {'every planted number caught' if ok else 'A PLANTED NUMBER WAS NOT CAUGHT'}; the real documents are "
                 f"{'clean' if real['status'] == 'PASS' else 'NOT clean (see the baseline)'}, which is a separate question")
    return ok, lines


# --- main ------------------------------------------------------------------------------------------------------------------------

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--repo-root", default=REPO)
    parser.add_argument("--ledger", default=DEFAULT_LEDGER)
    parser.add_argument("--doc", action="append", help="a document to scan, relative to the repository (default docs/pr/PR14_BODY.md)")
    parser.add_argument("--report", action="append", default=[], help="a generated report to scan in addition to the newest committed ones")
    parser.add_argument("--no-default-reports", action="store_true")
    parser.add_argument("--json", metavar="OUT")
    parser.add_argument("--verbose", "-v", action="store_true", help="print the backed claims too")
    parser.add_argument("--skeleton", action="store_true")
    parser.add_argument("--canary", action="store_true")
    args = parser.parse_args(argv)
    try:
        if args.canary:
            ok, lines = canary(args.repo_root, args.ledger)
            print("\n".join(lines))
            return 0 if ok else 1
        docs = args.doc or DEFAULT_DOCS
        defaults = [] if args.no_default_reports else newest_reports(args.repo_root)
        report = run(args.repo_root, args.ledger, docs, defaults, args.report)
    except SystemExit_ as exc:
        print(f"check_evidence_backing: {exc}", file=sys.stderr)
        return exc.code
    if args.skeleton:
        print(skeleton(report))
        return 0
    print(format_report(report, args.verbose))
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2)
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
