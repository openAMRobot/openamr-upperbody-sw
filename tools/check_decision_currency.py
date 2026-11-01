#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Decision-currency check (P22 task C, a prototype for this repository only).

Fails when one of our documents cites a decision at a revision that is not the one the register records as current.

    check_decision_currency.py --register PATH/decisions.yaml [--design-root DIR | --repo-only] [--json OUT]
    check_decision_currency.py --register PATH/decisions.yaml --discover      # list versioned citations found in the scanned documents
    check_decision_currency.py --register PATH/decisions.yaml --canary        # plant faults in temporary copies and show they are caught

What it compares. `docs/decision_citations.yaml` is a machine-readable list of (document, decision ID, revision cited). Each entry also
carries `quote`, a verbatim piece of the document that states the citation, so the list cannot drift away from the text: the quote must be
found in the document, and it must contain the decision ID and the revision the list says. The revision is then compared with the register:
the `source.document` of the decision's entry in `openAMRobot/.github` `decisions.yaml` is the revision that decision currently stands on.

  CURRENT      the document cites the revision the register records
  STALE        it cites an older one (FAIL)
  AHEAD        it cites a newer one than the register records: the register is behind, or the document is wrong (FAIL, for the owner to settle)
  SUPERSEDED   the decision is `status: superseded` in the register and the document does not say it is history (FAIL)
  HISTORICAL   the list says the citation is history, with a reason; the text must still be there, the revision is not compared
  UNKNOWN_*    the decision ID or the revision is not in the register (FAIL)
  QUOTE_*      the quote is not in the document, or does not state the ID or the revision the list says (FAIL)
  UNLISTED     a living document names a hyphenated register ID next to a revision and the list has no entry for it (FAIL)

What it does not do. It reads the register and our files and changes nothing. It proves that the revision a document names is the one the
register names, not that the value the document states is the register's value (the register's own checker, tools/check_decisions.py in
openAMRobot/.github, scans for contradicting phrases). A citation with no revision in its text cannot be stale and is only counted
(UNVERSIONED). A non-hyphenated ID (LIFT, CAMERAS) is checked when listed and found by the unlisted scan only through --discover's hint.
It is not wired into any CI; the register comes from a path, and which commit of the harness repository that is gets printed.

Exit status: 0 nothing wrong; 1 at least one FAIL; 2 the register, the list or an argument cannot be read; 3 some entries could not be
checked (design documents without --design-root; pass --repo-only to say that on purpose) and nothing failed.
"""
import argparse
import json
import os
import re
import subprocess
import sys
import tempfile
from fnmatch import fnmatch

try:
    import yaml
except ImportError:                                     # pragma: no cover
    sys.exit("check_decision_currency: PyYAML is required (python3-yaml)")

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
DEFAULT_CITATIONS = os.path.join(REPO, "docs", "decision_citations.yaml")
DESIGN_PREFIX = "design:"
ALLOW_MARKER = "decision-allow"

FAIL_STATES = {"STALE", "AHEAD", "SUPERSEDED", "UNKNOWN_DECISION", "UNKNOWN_REVISION", "QUOTE_NOT_FOUND", "QUOTE_MISMATCH",
               "DOC_MISSING", "UNLISTED", "BAD_ENTRY"}

MONTHS = {m: i for i, m in enumerate(
    ["jan", "feb", "mar", "apr", "may", "jun", "jul", "aug", "sep", "oct", "nov", "dec"], 1)}


# --- text ----------------------------------------------------------------------------------------------------------------

def squash(text):
    """(squashed, line_of_char): comment and quote markers at the start of a line, emphasis and code marks dropped, whitespace collapsed.
    The same function is applied to a quote and to the document, so a quote copied from a wrapped Markdown or YAML comment still matches."""
    out, lines = [], []
    for number, raw in enumerate(text.split("\n"), 1):
        line = re.sub(r"^[\s>#]+", "", raw)
        line = line.replace("**", "").replace("`", "").replace("*", "")
        line = re.sub(r"\s+", " ", line).strip()
        if not line:
            continue
        piece = line + " "
        out.append(piece)
        lines.extend([number] * len(piece))
    return "".join(out), lines


def find_span(text, quote):
    """(first line, last line), 1-based, of `quote` in `text` after squashing both, or None."""
    squashed, lines = squash(text)
    needle = squash(quote)[0].strip()
    if not needle:
        return None
    at = squashed.find(needle)
    return (lines[at], lines[at + len(needle) - 1]) if at >= 0 else None


def parse_rev_key(key):
    """('P-03', (19, 4)) for 'P-03-rev19.4', ('E-01', (1, 3)), ('BOM', (7, 10)), ('Alex', (2026, 10, 16)), or None."""
    m = re.fullmatch(r"(P-\d\d|E-\d\d)-rev(\d+)\.(\d+)", key)
    if m:
        return m.group(1), (int(m.group(2)), int(m.group(3)))
    m = re.fullmatch(r"BOM-Issue-(\d+)(?:\.(\d+))?", key)
    if m:
        return "BOM", (int(m.group(1)), int(m.group(2) or 0))
    m = re.fullmatch(r"Alex decision (\d{4})-(\d\d)-(\d\d)", key)
    if m:
        return "Alex", tuple(int(g) for g in m.groups())
    return None


def revision_in_text(key, text):
    """Does `text` state the revision named by the register source key `key`?"""
    parsed = parse_rev_key(key)
    if parsed is None:
        return key in text
    family, number = parsed
    if family in ("P-03", "P-00"):
        major, minor = number
        return re.search(r"rev(?:ision)?\.?\s*%d\.%d\b" % (major, minor), text, re.I) is not None
    if family == "E-01":
        major, minor = number
        return re.search(r"E-01\W{0,4}rev(?:ision)?\.?\s*%d\.%d\b" % (major, minor), text, re.I) is not None
    if family == "BOM":
        major, minor = number
        pattern = r"Issue\s*%d(?:\.%d)?\b" % (major, minor) if minor else r"Issue\s*%d\b" % major
        return re.search(pattern, text, re.I) is not None
    year, month, day = number
    if re.search(r"%04d-%02d-%02d" % number, text):
        return True
    for name, index in MONTHS.items():
        if index == month and re.search(r"\b0?%d\s+%s[a-z]*\.?,?\s+%d\b" % (day, name, year), text, re.I):
            return True
    return False


REV_TOKEN = [
    ("E-01", re.compile(r"E-01\W{0,4}rev(?:ision)?\.?\s*(\d+)\.(\d+)", re.I)),
    ("P-03", re.compile(r"(?<![\w-])(?:P-03\W{0,4})?rev(?:ision)?\.?\s*(\d+)\.(\d+)\b", re.I)),
    ("BOM", re.compile(r"BOM\s+Issue\s+(\d+)(?:\.(\d+))?", re.I)),
]
ALEX_TOKEN = re.compile(r"Alex(?:'s)? decision\W{0,3}(?:of\s+)?(\d{4})-(\d\d)-(\d\d)|Alex(?:'s)? decision\W{0,3}(?:of\s+)?(\d{1,2})\s+([A-Za-z]{3})[a-z]*\.?,?\s+(\d{4})", re.I)


def revision_tokens(text):
    """Register source keys that `text` names (P-03 revisions, E-01 revisions, BOM issues, Alex decisions by date)."""
    found = set()
    for family, regex in REV_TOKEN:
        for m in regex.finditer(text):
            if family == "BOM":
                found.add("BOM-Issue-%s" % (m.group(1) + ("." + m.group(2) if m.group(2) else "")))
            else:
                found.add("%s-rev%s.%s" % (family, m.group(1), m.group(2)))
    for m in ALEX_TOKEN.finditer(text):
        if m.group(1):
            found.add("Alex decision %s-%s-%s" % m.group(1, 2, 3))
        elif m.group(5).lower()[:3] in MONTHS:
            found.add("Alex decision %s-%02d-%02d" % (m.group(6), MONTHS[m.group(5).lower()[:3]], int(m.group(4))))
    return found


# --- the register ------------------------------------------------------------------------------------------------------------

class Register:
    def __init__(self, path):
        self.path = path
        try:
            with open(path, encoding="utf-8") as f:
                doc = yaml.safe_load(f)
        except (OSError, yaml.YAMLError) as exc:
            raise SystemExit_(2, f"cannot read the register {path}: {exc}")
        if not isinstance(doc, dict) or not isinstance(doc.get("decisions"), list) or not isinstance(doc.get("sources"), dict):
            raise SystemExit_(2, f"{path} is not a decision register (no `decisions` list or no `sources` map)")
        self.sources = set(doc["sources"])
        self.in_force = (doc.get("in_force") or {}).get("source")
        self.entries = {}
        for entry in doc["decisions"]:
            if isinstance(entry, dict) and entry.get("id"):
                self.entries[entry["id"]] = entry
        self.sha = self._sha()

    def _sha(self):
        directory = os.path.dirname(os.path.abspath(self.path))
        head = subprocess.run(["git", "-C", directory, "rev-parse", "HEAD"], capture_output=True, text=True)
        if head.returncode != 0:
            return None
        dirty = subprocess.run(["git", "-C", directory, "status", "--porcelain", "--", os.path.basename(self.path)],
                               capture_output=True, text=True).stdout.strip()
        return head.stdout.strip() + (" (register file modified)" if dirty else "")

    def current_revision(self, decision_id):
        source = self.entries[decision_id].get("source") or {}
        return source.get("document") if isinstance(source, dict) else None

    def hyphenated_ids(self):
        return sorted((i for i in self.entries if "-" in i), key=len, reverse=True)


class SystemExit_(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


# --- the list of citations -----------------------------------------------------------------------------------------------------

def load_citations(path):
    try:
        with open(path, encoding="utf-8") as f:
            doc = yaml.safe_load(f)
    except (OSError, yaml.YAMLError) as exc:
        raise SystemExit_(2, f"cannot read the citation list {path}: {exc}")
    if not isinstance(doc, dict) or not isinstance(doc.get("citations"), list):
        raise SystemExit_(2, f"{path}: expected a mapping with a `citations` list")
    return doc


def resolve_document(name, repo_root, design_root):
    """(path, None) | (None, reason). `design:` documents live in the design folder, outside this repository."""
    if name.startswith(DESIGN_PREFIX):
        if not design_root:
            return None, "NOT_RUN"
        return os.path.join(design_root, name[len(DESIGN_PREFIX):]), None
    return os.path.join(repo_root, name), None


def read_text(path):
    try:
        with open(path, encoding="utf-8") as f:
            return f.read()
    except OSError:
        return None


# --- the check -------------------------------------------------------------------------------------------------------------------

def check_citation(register, entry, text):
    """(state, message) for one listed citation whose document text is `text`."""
    decision, revision, quote = entry.get("decision"), entry.get("revision"), entry.get("quote")
    if not (decision and revision and quote):
        return "BAD_ENTRY", "a citation needs `document`, `decision`, `revision` and `quote`"
    if decision not in register.entries:
        return "UNKNOWN_DECISION", f"{decision} is not a decision of the register"
    if revision not in register.sources:
        return "UNKNOWN_REVISION", f"{revision} is not a source of the register"
    span = find_span(text, quote)
    if span is None:
        return "QUOTE_NOT_FOUND", "the quoted citation is not in the document (the list is out of date, or the text was reworded)"
    squashed_quote = squash(quote)[0]
    if decision not in squashed_quote:
        return "QUOTE_MISMATCH", f"the quote does not name {decision}"
    if not revision_in_text(revision, squashed_quote):
        return "QUOTE_MISMATCH", f"the quote does not state {revision}"
    entry["_line"], entry["_span"] = span[0], span
    historical = entry.get("historical")
    if historical:
        return "HISTORICAL", f"kept as history: {historical}"
    register_entry = register.entries[decision]
    current = register.current_revision(decision)
    if register_entry.get("status") == "superseded":
        return "SUPERSEDED", (f"{decision} is superseded in the register (by {((register_entry.get('superseded_by') or {}).get('decision')) or 'a later decision'}); "
                              "the document cites it as current")
    if current is None:
        return "UNKNOWN_REVISION", f"{decision} has no source document in the register"
    if revision == current:
        return "CURRENT", f"{decision} at {revision}"
    cited, standing = parse_rev_key(revision), parse_rev_key(current)
    if cited and standing and cited[0] == standing[0]:
        if cited[1] < standing[1]:
            return "STALE", f"cites {revision}; the register records {current}"
        return "AHEAD", f"cites {revision}; the register records {current} (the register is behind, or the document is wrong)"
    return "STALE", f"cites {revision}; the register records {current}"


def scan_unlisted(register, document, text, covered):
    """Versioned mentions of hyphenated register IDs in a living document that no listed citation covers: `covered` is {decision: [(first, last)]},
    the lines of the quotes of the list's entries, widened by one line each side. -> (unlisted, unversioned_count)."""
    ids = register.hyphenated_ids()
    if not ids:
        return [], 0
    regex = re.compile(r"(?<![A-Za-z0-9_-])(%s)(?![A-Za-z0-9_-])" % "|".join(re.escape(i) for i in ids))
    lines = text.split("\n")
    unlisted, unversioned = [], 0
    seen = set()
    for index, raw in enumerate(lines):
        if ALLOW_MARKER in raw:
            continue
        for m in regex.finditer(raw):
            decision = m.group(1)
            window = "\n".join(lines[max(0, index - 1):index + 2])
            if not revision_tokens(squash(window)[0]):
                unversioned += 1
                continue
            if any(first - 1 <= index + 1 <= last + 1 for first, last in covered.get(decision, [])) or (decision, index + 1) in seen:
                continue
            seen.add((decision, index + 1))
            unlisted.append({"document": document, "decision": decision, "line": index + 1,
                             "revisions_nearby": sorted(revision_tokens(squash(window)[0]))})
    return unlisted, unversioned


def run(register, citations_doc, repo_root, design_root, repo_only):
    """The whole check. -> report dict."""
    results, unlisted, unversioned, not_run = [], [], 0, []
    covered = {}                                    # {document: {decision: [(first, last)]}}
    for entry in citations_doc["citations"]:
        entry = dict(entry)
        document = entry.get("document", "")
        path, why = resolve_document(document, repo_root, design_root)
        record = {"document": document, "decision": entry.get("decision"), "revision": entry.get("revision")}
        if path is None:
            if repo_only:
                record.update(state="SKIPPED", message="design document, --repo-only given")
            else:
                record.update(state="NOT_RUN", message="design document and no --design-root")
                not_run.append(record)
            results.append(record)
            continue
        text = read_text(path)
        if text is None:
            record.update(state="DOC_MISSING", message=f"{document} cannot be read")
        else:
            state, message = check_citation(register, entry, text)
            record.update(state=state, message=message, line=entry.get("_line"))
            if entry.get("_span"):
                covered.setdefault(document, {}).setdefault(entry.get("decision"), []).append(entry["_span"])
            if state in ("STALE", "AHEAD", "SUPERSEDED"):
                record["register_revision"] = register.current_revision(entry["decision"])
        results.append(record)
    scan = citations_doc.get("scan") or {}
    for scope, root, globs in (("repo", repo_root, scan.get("repo") or []), ("design", design_root, scan.get("design") or [])):
        if scope == "design" and not design_root:
            continue
        for relative in expand_globs(root, globs):
            document = relative if scope == "repo" else DESIGN_PREFIX + relative
            text = read_text(os.path.join(root, relative))
            if text is None:
                continue
            found, count = scan_unlisted(register, document, text, covered.get(document, {}))
            unlisted.extend(found)
            unversioned += count
    for item in unlisted:
        results.append({"document": item["document"], "decision": item["decision"], "revision": None, "state": "UNLISTED", "line": item["line"],
                        "message": f"names {item['decision']} next to {', '.join(item['revisions_nearby'])}; the list has no entry for this document and decision"})
    failed = [r for r in results if r["state"] in FAIL_STATES]
    if failed:
        status = "FAIL"
    elif not_run:
        status = "NOT_RUN"
    else:
        status = "PASS"
    return {"schema": "openamr.decision_currency.v1", "register": register.path, "register_sha": register.sha,
            "register_in_force": register.in_force, "status": status, "results": results,
            "counts": count_states(results), "unversioned_mentions": unversioned}


def expand_globs(root, patterns):
    if not root or not os.path.isdir(root):
        return []
    found = []
    for base, dirs, files in os.walk(root):
        dirs[:] = [d for d in dirs if d not in (".git", "node_modules", "build", "install", "log", "__pycache__", ".pytest_cache")]
        for name in files:
            relative = os.path.relpath(os.path.join(base, name), root).replace(os.sep, "/")
            if any(fnmatch(relative, pattern) for pattern in patterns):          # anchored: "README.md" is the top-level one only
                found.append(relative)
    return sorted(found)


def count_states(results):
    counts = {}
    for r in results:
        counts[r["state"]] = counts.get(r["state"], 0) + 1
    return counts


def format_report(report):
    lines = [f"register: {report['register']} @ {report['register_sha'] or 'no git checkout (commit unknown)'}; in force: {report['register_in_force']}"]
    for r in sorted(report["results"], key=lambda r: (r["state"] not in FAIL_STATES, r["document"], r.get("line") or 0)):
        where = f"{r['document']}:{r['line']}" if r.get("line") else r["document"]
        lines.append(f"{r['state']:<16} {where}  {r['decision']}  {r['message']}")
    summary = ", ".join(f"{k} {v}" for k, v in sorted(report["counts"].items()))
    lines.append(f"{report['status']}: {summary}; {report['unversioned_mentions']} mentions of a hyphenated ID with no revision nearby (not checkable)")
    return "\n".join(lines)


# --- discover -------------------------------------------------------------------------------------------------------------------

def discover(register, citations_doc, repo_root, design_root):
    """Print the versioned mentions of hyphenated IDs in the scanned documents as entries ready to review. Not a verdict."""
    scan = citations_doc.get("scan") or {}
    out = []
    for scope, root, globs in (("repo", repo_root, scan.get("repo") or []), ("design", design_root, scan.get("design") or [])):
        for relative in expand_globs(root, globs):
            document = relative if scope == "repo" else DESIGN_PREFIX + relative
            text = read_text(os.path.join(root, relative))
            if text is None:
                continue
            found, _ = scan_unlisted(register, document, text, {})
            for item in found:
                out.append(f"- {{document: {item['document']}, decision: {item['decision']}, line: {item['line']}, revisions_nearby: {item['revisions_nearby']}}}")
    return "\n".join(out)


# --- canary -------------------------------------------------------------------------------------------------------------------------

def canary(register, citations_doc, repo_root, design_root):
    """Plant faults in temporary copies, never in the tracked files, and show that each is caught and the unplanted copy is clean.
    Returns (ok, lines)."""
    lines, ok = [], True
    base = run(register, citations_doc, repo_root, design_root, repo_only=not design_root)
    clean = base["status"] == "PASS"
    lines.append(f"baseline (the real list against the real documents): {base['status']}   {base['counts']}")
    # Work on a copy of the repo documents the list names, so a planted sentence is a sentence in a document.
    with tempfile.TemporaryDirectory(prefix="decision_currency_canary_") as tmp:
        # A hyphenated ID, because only those are found by the unlisted scan; the decision is taken from the register itself when the
        # real list has none that is current (an empty or all-stale list must not make the canary unrunnable).
        decision = next((e["decision"] for e in citations_doc["citations"] if not e.get("historical") and "-" in str(e.get("decision"))
                         and register.entries.get(e["decision"]) and register.current_revision(e["decision"]) == e.get("revision")), None)
        if decision is None:
            decision = next((i for i in register.hyphenated_ids() if parse_rev_key(register.current_revision(i) or "")
                             and parse_rev_key(register.current_revision(i))[0] == "P-03"), None)
        if decision is None:
            return False, lines + ["no hyphenated decision on a P-03 revision in the register to plant a fault beside; the canary cannot run"]
        current = register.current_revision(decision)
        older = pick_older_revision(register, current)
        if older is None:
            return False, lines + [f"no older P-03 revision than {current} in the register's sources to plant"]
        quote_old = f"{decision} (P-03 {rev_text(older)} item 19)"
        planted_doc = os.path.join(tmp, "PLANTED.md")
        with open(planted_doc, "w", encoding="utf-8") as f:
            f.write(f"# planted\n\nThe frame is placed as in {quote_old}.\n")
        cases = [
            ("a document cites an older revision of a decision", [{"document": "PLANTED.md", "decision": decision, "revision": older, "quote": quote_old}], "STALE", planted_doc, None),
            ("the list names a decision the register does not have", [{"document": "PLANTED.md", "decision": "NO-SUCH-DECISION", "revision": current, "quote": quote_old}], "UNKNOWN_DECISION", planted_doc, None),
            ("the quote is not in the document", [{"document": "PLANTED.md", "decision": decision, "revision": older, "quote": quote_old + " and more words that are not there"}], "QUOTE_NOT_FOUND", planted_doc, None),
            ("the list says one revision and the quote states another", [{"document": "PLANTED.md", "decision": decision, "revision": current, "quote": quote_old}], "QUOTE_MISMATCH", planted_doc, None),
            ("a document cites a decision and revision the list does not have", [], "UNLISTED", planted_doc, {"repo": ["PLANTED.md"]}),
        ]
        for label, entries, expect, doc_path, scan in cases:
            if entries and entries[0]["revision"] == current and expect == "QUOTE_MISMATCH" and revision_in_text(current, squash(quote_old)[0]):
                lines.append(f"cannot plant '{label}': the old and the current revision read alike")
                ok = False
                continue
            planted = {"schema_version": 1, "citations": entries, "scan": scan or {}}
            result = run(register, planted, tmp, None, repo_only=True)
            states = {r["state"] for r in result["results"]}
            caught = expect in states and result["status"] == "FAIL"
            ok &= caught
            lines.append(f"{'CAUGHT ' if caught else 'MISSED '} {label}: expected {expect}, got {sorted(states) or 'nothing'}, status {result['status']}")
        # The same fault in a REAL document of this repository: copy it, re-date one real current citation in the copy and in the list
        # together (the way a careless edit would), and run. The tracked file is read, never written.
        planted_real = None
        for e in citations_doc["citations"]:
            if e["document"].startswith(DESIGN_PREFIX) or e.get("historical") or not register.entries.get(e.get("decision")):
                continue
            parsed = parse_rev_key(e.get("revision", ""))
            if register.current_revision(e["decision"]) != e["revision"] or not parsed or parsed[0] != "P-03":
                continue
            text = read_text(os.path.join(repo_root, e["document"]))
            if text is not None and e["quote"] in text and re.search(r"rev(?:ision)?\s*\d+\.\d+", e["quote"]) and pick_older_revision(register, e["revision"]):
                planted_real = (e, text)                # the quote sits on one line of the raw file, so it can be replaced as it stands
                break
        if planted_real is None:
            lines.append("no current repo citation whose quote is verbatim on one line of its document; the real-document canary was not run")
            ok = False
        else:
            e, text = planted_real
            older_real = pick_older_revision(register, e["revision"])
            number = "%d.%d" % parse_rev_key(older_real)[1]
            new_quote = re.sub(r"(rev(?:ision)?\s*)\d+\.\d+", lambda m: m.group(1) + number, e["quote"], count=1)
            copy_path = os.path.join(tmp, e["document"])
            os.makedirs(os.path.dirname(copy_path), exist_ok=True)
            with open(copy_path, "w", encoding="utf-8") as f:
                f.write(text.replace(e["quote"], new_quote, 1))
            entry = {"document": e["document"], "decision": e["decision"], "revision": older_real, "quote": new_quote}
            result = run(register, {"schema_version": 1, "citations": [entry], "scan": {}}, tmp, None, repo_only=True)
            states = {r["state"] for r in result["results"]}
            caught = "STALE" in states and result["status"] == "FAIL"
            ok &= caught
            lines.append(f"{'CAUGHT ' if caught else 'MISSED '} a copy of {e['document']} with its citation of {e['decision']} re-dated "
                         f"{e['revision']} -> {older_real}, document and list edited together: statuses {sorted(states)}")
        clean_text = f"# clean\n\nThe frame is placed as in {decision} (P-03 {rev_text(current)} item 19).\n"
        with open(planted_doc, "w", encoding="utf-8") as f:
            f.write(clean_text)
        current_entry = {"document": "PLANTED.md", "decision": decision, "revision": current, "quote": f"{decision} (P-03 {rev_text(current)} item 19)"}
        result = run(register, {"schema_version": 1, "citations": [current_entry], "scan": {"repo": ["PLANTED.md"]}}, tmp, None, repo_only=True)
        lines.append(f"{'OK     ' if result['status'] == 'PASS' else 'WRONG  '} the same sentence at the current revision is clean: {result['status']}")
        ok &= result["status"] == "PASS"
    lines.append(f"canary: {'every planted fault caught' if ok else 'A PLANTED FAULT WAS NOT CAUGHT'}; the real list is {'clean' if clean else 'NOT clean (see the baseline)'}, which is a separate question")
    return ok, lines


def rev_text(key):
    parsed = parse_rev_key(key)
    return "rev%d.%d" % parsed[1] if parsed else key


def pick_older_revision(register, current):
    parsed = parse_rev_key(current)
    if parsed is None or parsed[0] != "P-03":
        return None
    older = [s for s in register.sources if parse_rev_key(s) and parse_rev_key(s)[0] == "P-03" and parse_rev_key(s)[1] < parsed[1]]
    return max(older, key=lambda s: parse_rev_key(s)[1]) if older else None


# --- main ---------------------------------------------------------------------------------------------------------------------------

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--register", default=os.environ.get("OPENAMR_DECISIONS_YAML"), help="path to openAMRobot/.github decisions.yaml (or OPENAMR_DECISIONS_YAML)")
    parser.add_argument("--citations", default=DEFAULT_CITATIONS)
    parser.add_argument("--repo-root", default=REPO)
    parser.add_argument("--design-root", default=os.environ.get("OPENAMR_DESIGN_DIR"), help="the design folder's 10_architecture directory (or OPENAMR_DESIGN_DIR)")
    parser.add_argument("--repo-only", action="store_true", help="do not check the design folder's documents, on purpose")
    parser.add_argument("--json", metavar="OUT", help="write the report as JSON")
    parser.add_argument("--discover", action="store_true")
    parser.add_argument("--canary", action="store_true")
    args = parser.parse_args(argv)
    if not args.register:
        print("check_decision_currency: --register (or OPENAMR_DECISIONS_YAML) is required", file=sys.stderr)
        return 2
    try:
        register = Register(args.register)
        citations_doc = load_citations(args.citations)
    except SystemExit_ as exc:
        print(f"check_decision_currency: {exc}", file=sys.stderr)
        return exc.code
    design_root = None if args.repo_only else args.design_root
    if args.discover:
        print(discover(register, citations_doc, args.repo_root, design_root))
        return 0
    if args.canary:
        ok, lines = canary(register, citations_doc, args.repo_root, design_root)
        print("\n".join(lines))
        return 0 if ok else 1
    report = run(register, citations_doc, args.repo_root, design_root, args.repo_only)
    print(format_report(report))
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2)
    return {"PASS": 0, "FAIL": 1, "NOT_RUN": 3}[report["status"]]


if __name__ == "__main__":
    sys.exit(main())
