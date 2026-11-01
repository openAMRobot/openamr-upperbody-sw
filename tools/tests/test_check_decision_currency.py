# SPDX-License-Identifier: MIT
"""tools/check_decision_currency.py against a small register of its own (nothing here reads the real decisions.yaml)."""
import json

import pytest
import yaml

import check_decision_currency as cdc

REGISTER = {
    "schema_version": 1,
    "in_force": {"source": "P-03-rev19.6"},
    "sources": {k: {"title": k} for k in ["P-03-rev18.7", "P-03-rev18.9", "P-03-rev19.0", "P-03-rev19.2", "P-03-rev19.4", "P-03-rev19.6",
                                          "Alex decision 2026-10-12", "BOM-Issue-7.10"]},
    "decisions": [
        {"id": "FRAMES-REP105", "status": "recorded", "source": {"document": "P-03-rev19.4", "item": "x"}},
        {"id": "NEAR-FIELD-FRAMES", "status": "recorded", "source": {"document": "P-03-rev19.0", "item": "x"}},
        {"id": "CAMERAS", "status": "recorded", "source": {"document": "P-03-rev19.4", "item": "x"}},
        {"id": "MAST-POSITIONS", "status": "superseded", "source": {"document": "P-03-rev18.7", "item": "x"}, "superseded_by": {"decision": "LIFT"}},
        {"id": "LIFT", "status": "recorded", "source": {"document": "P-03-rev18.9", "item": "x"}},
    ],
}


@pytest.fixture
def register(tmp_path):
    path = tmp_path / "decisions.yaml"
    path.write_text(yaml.safe_dump(REGISTER))
    return cdc.Register(str(path))


def check(register, tmp_path, text, entry, scan=None):
    (tmp_path / "doc.md").write_text(text)
    entry = {"document": "doc.md", **entry}
    return cdc.run(register, {"schema_version": 1, "citations": [entry], "scan": scan or {}}, str(tmp_path), None, True)


def states(report):
    return sorted(r["state"] for r in report["results"])


def test_a_citation_at_the_revision_the_register_records_is_current(register, tmp_path):
    report = check(register, tmp_path, "The IMU is placed as in FRAMES-REP105 (P-03 rev19.4).\n",
                   {"decision": "FRAMES-REP105", "revision": "P-03-rev19.4", "quote": "FRAMES-REP105 (P-03 rev19.4)"})
    assert states(report) == ["CURRENT"] and report["status"] == "PASS"


def test_an_older_revision_is_stale_and_fails(register, tmp_path):
    report = check(register, tmp_path, "Placed as in FRAMES-REP105 (P-03 rev19.0).\n",
                   {"decision": "FRAMES-REP105", "revision": "P-03-rev19.0", "quote": "FRAMES-REP105 (P-03 rev19.0)"})
    assert states(report) == ["STALE"] and report["status"] == "FAIL"
    assert report["results"][0]["register_revision"] == "P-03-rev19.4"


def test_a_newer_revision_than_the_register_records_is_ahead_and_fails(register, tmp_path):
    report = check(register, tmp_path, "Placed as in NEAR-FIELD-FRAMES (P-03 rev19.4).\n",
                   {"decision": "NEAR-FIELD-FRAMES", "revision": "P-03-rev19.4", "quote": "NEAR-FIELD-FRAMES (P-03 rev19.4)"})
    assert states(report) == ["AHEAD"] and report["status"] == "FAIL"


def test_a_citation_of_another_family_is_stale(register, tmp_path):
    report = check(register, tmp_path, "since FRAMES-REP105 (Alex decision 2026-10-12)\n",
                   {"decision": "FRAMES-REP105", "revision": "Alex decision 2026-10-12", "quote": "FRAMES-REP105 (Alex decision 2026-10-12)"})
    assert states(report) == ["STALE"]


def test_an_unknown_decision_or_revision_fails(register, tmp_path):
    text = "NO-SUCH (P-03 rev19.4) and FRAMES-REP105 (P-03 rev1.1)\n"
    assert states(check(register, tmp_path, text, {"decision": "NO-SUCH", "revision": "P-03-rev19.4", "quote": "NO-SUCH (P-03 rev19.4)"})) == ["UNKNOWN_DECISION"]
    assert states(check(register, tmp_path, text, {"decision": "FRAMES-REP105", "revision": "P-03-rev1.1", "quote": "FRAMES-REP105 (P-03 rev1.1)"})) == ["UNKNOWN_REVISION"]


def test_the_quote_must_be_in_the_document_and_state_the_id_and_the_revision(register, tmp_path):
    entry = {"decision": "FRAMES-REP105", "revision": "P-03-rev19.4"}
    text = "Placed as in FRAMES-REP105 (P-03 rev19.0).\n"
    assert states(check(register, tmp_path, text, {**entry, "quote": "FRAMES-REP105 (P-03 rev19.4)"})) == ["QUOTE_NOT_FOUND"]
    assert states(check(register, tmp_path, text, {**entry, "quote": "FRAMES-REP105 (P-03 rev19.0)"})) == ["QUOTE_MISMATCH"]
    assert states(check(register, tmp_path, "see (P-03 rev19.4) there\n", {**entry, "quote": "(P-03 rev19.4)"})) == ["QUOTE_MISMATCH"]


def test_a_quote_survives_line_wraps_comment_marks_and_emphasis(register, tmp_path):
    text = "# Placed since 935efc7 (Alex decision 2026-10-12;\n# register **FRAMES-REP105**, current at P-03\n# rev19.4) the root is x\n"
    report = check(register, tmp_path, text, {"decision": "FRAMES-REP105", "revision": "P-03-rev19.4",
                                              "quote": "register FRAMES-REP105, current at P-03 rev19.4"})
    assert states(report) == ["CURRENT"] and report["results"][0]["line"] == 2


def test_a_superseded_decision_cited_as_current_fails(register, tmp_path):
    report = check(register, tmp_path, "mast positions MAST-POSITIONS (P-03 rev18.7) apply\n",
                   {"decision": "MAST-POSITIONS", "revision": "P-03-rev18.7", "quote": "MAST-POSITIONS (P-03 rev18.7)"})
    assert states(report) == ["SUPERSEDED"] and "LIFT" in report["results"][0]["message"]


def test_history_is_kept_with_a_reason_and_still_needs_its_text(register, tmp_path):
    entry = {"decision": "FRAMES-REP105", "revision": "P-03-rev18.7", "historical": "written against rev18.7",
             "quote": "FRAMES-REP105 (P-03 rev18.7)"}
    assert states(check(register, tmp_path, "Register FRAMES-REP105 (P-03 rev18.7) requires\n", entry)) == ["HISTORICAL"]
    assert states(check(register, tmp_path, "reworded\n", entry)) == ["QUOTE_NOT_FOUND"]


def test_a_missing_document_and_a_bad_entry_fail(register, tmp_path):
    (tmp_path / "here.md").write_text("LIFT (P-03 rev18.9)\n")
    report = cdc.run(register, {"citations": [{"document": "gone.md", "decision": "LIFT", "revision": "P-03-rev18.9", "quote": "LIFT (P-03 rev18.9)"},
                                              {"document": "here.md", "decision": "LIFT"}]}, str(tmp_path), None, True)
    assert states(report) == ["BAD_ENTRY", "DOC_MISSING"] and report["status"] == "FAIL"


def test_the_scan_finds_a_versioned_mention_the_list_lacks(register, tmp_path):
    (tmp_path / "doc.md").write_text("The frame follows FRAMES-REP105 (P-03 rev19.0).\n" + "filler\n" * 4 + "Elsewhere FRAMES-REP105 is named alone.\n")
    report = cdc.run(register, {"citations": [], "scan": {"repo": ["doc.md"]}}, str(tmp_path), None, True)
    assert states(report) == ["UNLISTED"] and report["results"][0]["line"] == 1
    assert report["unversioned_mentions"] == 1 and report["status"] == "FAIL"


def test_the_scan_is_line_aware_and_honours_the_allow_marker(register, tmp_path):
    text = ("First FRAMES-REP105 (P-03 rev19.4).\n" + "filler\n" * 5 +
            "Second FRAMES-REP105 (P-03 rev19.0).\n" +
            "Third FRAMES-REP105 (P-03 rev18.7) <!-- decision-allow: FRAMES-REP105 history -->\n")
    (tmp_path / "doc.md").write_text(text)
    listed = {"document": "doc.md", "decision": "FRAMES-REP105", "revision": "P-03-rev19.4", "quote": "First FRAMES-REP105 (P-03 rev19.4)"}
    report = cdc.run(register, {"citations": [listed], "scan": {"repo": ["doc.md"]}}, str(tmp_path), None, True)
    unlisted = [r for r in report["results"] if r["state"] == "UNLISTED"]
    assert [r["line"] for r in unlisted] == [7]            # the second mention only: the first is covered, the third is allowed


def test_design_documents_are_not_run_without_a_design_root_unless_asked(register, tmp_path):
    entry = {"document": "design:FRAMES.md", "decision": "LIFT", "revision": "P-03-rev18.9", "quote": "LIFT (P-03 rev18.9)"}
    report = cdc.run(register, {"citations": [entry]}, str(tmp_path), None, False)
    assert report["status"] == "NOT_RUN"
    assert cdc.run(register, {"citations": [entry]}, str(tmp_path), None, True)["status"] == "PASS"
    (tmp_path / "FRAMES.md").write_text("LIFT (P-03 rev18.9)\n")
    assert cdc.run(register, {"citations": [entry]}, str(tmp_path), str(tmp_path), False)["status"] == "PASS"


def test_the_cli_exit_codes(tmp_path, capsys):
    reg = tmp_path / "decisions.yaml"
    reg.write_text(yaml.safe_dump(REGISTER))
    (tmp_path / "doc.md").write_text("FRAMES-REP105 (P-03 rev19.0)\n")
    lst = tmp_path / "list.yaml"
    lst.write_text(yaml.safe_dump({"citations": [{"document": "doc.md", "decision": "FRAMES-REP105", "revision": "P-03-rev19.0",
                                                  "quote": "FRAMES-REP105 (P-03 rev19.0)"}]}))
    out = tmp_path / "out.json"
    argv = ["--register", str(reg), "--citations", str(lst), "--repo-root", str(tmp_path), "--repo-only", "--json", str(out)]
    assert cdc.main(argv) == 1 and json.loads(out.read_text())["status"] == "FAIL"
    assert cdc.main(["--citations", str(lst)]) == 2
    assert cdc.main(["--register", str(tmp_path / "missing.yaml"), "--citations", str(lst)]) == 2
    lst.write_text(yaml.safe_dump({"citations": [{"document": "doc.md", "decision": "FRAMES-REP105", "revision": "P-03-rev19.4",
                                                  "quote": "FRAMES-REP105 (P-03 rev19.0)"}]}))
    assert cdc.main(argv) == 1                                # the list says rev19.4, the text says rev19.0
    (tmp_path / "doc.md").write_text("FRAMES-REP105 (P-03 rev19.4)\n")
    lst.write_text(yaml.safe_dump({"citations": [{"document": "doc.md", "decision": "FRAMES-REP105", "revision": "P-03-rev19.4",
                                                  "quote": "FRAMES-REP105 (P-03 rev19.4)"}]}))
    assert cdc.main(argv) == 0


def test_the_canary_catches_every_planted_fault_on_a_clean_list(register, tmp_path):
    (tmp_path / "doc.md").write_text("The frame follows FRAMES-REP105 (P-03 rev19.4) and says so.\n")
    citations = {"schema_version": 1, "scan": {"repo": ["doc.md"]},
                 "citations": [{"document": "doc.md", "decision": "FRAMES-REP105", "revision": "P-03-rev19.4",
                                "quote": "FRAMES-REP105 (P-03 rev19.4)"}]}
    ok, lines = cdc.canary(register, citations, str(tmp_path), None)
    assert ok, "\n".join(lines)
    assert sum(line.startswith("CAUGHT") for line in lines) == 6 and "baseline" in lines[0] and "PASS" in lines[0]


def test_the_canary_fails_when_the_check_is_blind(register, tmp_path, monkeypatch):
    (tmp_path / "doc.md").write_text("FRAMES-REP105 (P-03 rev19.4)\n")
    citations = {"schema_version": 1, "scan": {}, "citations": [{"document": "doc.md", "decision": "FRAMES-REP105", "revision": "P-03-rev19.4",
                                                                  "quote": "FRAMES-REP105 (P-03 rev19.4)"}]}
    monkeypatch.setattr(cdc, "FAIL_STATES", set())             # a check that has stopped failing anything
    ok, lines = cdc.canary(register, citations, str(tmp_path), None)
    assert not ok and any(line.startswith("MISSED") for line in lines)


def test_revision_keys_and_text(register):
    assert cdc.revision_tokens("P-03 rev19.4, and Alex decision 2026-10-16; later Alex's decision of 12 Oct 2026, BOM Issue 7.10") == {
        "P-03-rev19.4", "Alex decision 2026-10-16", "Alex decision 2026-10-12", "BOM-Issue-7.10"}
    assert cdc.revision_in_text("P-03-rev19.0", "rev 19.0 item 19") and not cdc.revision_in_text("P-03-rev19.0", "rev19.04")
    assert cdc.revision_in_text("Alex decision 2026-10-12", "on 12 October 2026") and not cdc.revision_in_text("Alex decision 2026-10-12", "13 Oct 2026")
