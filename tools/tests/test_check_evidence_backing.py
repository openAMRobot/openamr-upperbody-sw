# SPDX-License-Identifier: MIT
"""tools/check_evidence_backing.py on small trees of its own."""
import json
import os

import pytest
import yaml

import check_evidence_backing as ceb


def report(platform="abc1234", git="aaaaaaa", results=("PASS", "PASS", "WARN"), profile="a7", extra=None):
    doc = {"profile": profile, "git_hash": git * 6, "upstream": {"openamr-platform-sw": platform},
           "checks": [{"id": f"C{i}", "result": r} for i, r in enumerate(results)], "limitations": ["stands alone, no numbers here"]}
    doc.update(extra or {})
    return doc


@pytest.fixture
def tree(tmp_path):
    (tmp_path / "evidence" / "reports").mkdir(parents=True)
    (tmp_path / "docs" / "pr").mkdir(parents=True)

    def put(name, data):
        (tmp_path / "evidence" / "reports" / name).write_text(json.dumps(data))

    put("20260101T000000Z_a7.json", report())
    put("20260102T000000Z_a7.json", report(platform="abc1234", git="bbbbbbb", results=("PASS", "FAIL", "WARN")))
    put("20260103T000000Z_dev.json", report(profile="dev", results=("PASS",) * 4))
    (tmp_path / "evidence" / "reports" / "README.md").write_text("Written down: the bridge died in two of the 8 runs.\n")
    tree.root = tmp_path
    return tmp_path


def check(tree, body, ledger=(), reports=()):
    (tree / "docs" / "pr" / "BODY.md").write_text(body)
    path = tree / "ledger.yaml"
    path.write_text(yaml.safe_dump({"claims": list(ledger)}))
    return ceb.run(str(tree), str(path), ["docs/pr/BODY.md"], list(reports))


def statuses(result):
    return sorted(r["status"] for r in result["claims"])


def test_the_claim_grammar():
    found = lambda t: [(c, k) for c, k in ceb.find_claims(t)]
    assert found("25 PASS / 3 WARN (E01, W02, V01) / 0 FAIL / 0 NOT_RUN") == [("25 PASS / 3 WARN (E01, W02, V01) / 0 FAIL / 0 NOT_RUN", "verdicts")]
    assert found("table: 14 / 2 / 0 and 1795 / 957 / 1832") == [("14 / 2 / 0", "bare_triple"), ("1795 / 957 / 1832", "bare_triple")]
    assert found("2 of 36 and about 1 in 13") == [("2 of 36", "rate_of"), ("about 1 in 13", "rate_in")]
    assert found("Two of the eight A7 runs, one in 49 launches") == [("Two of the eight A7 runs", "rate_words"), ("one in 49 launches", "rate_words")]
    assert found("826 tests, 805 passed, 20%") == [("826 tests", "count"), ("805 passed", "count"), ("20%", "percent")]
    # not claims: decimals, a number word without a count noun, a size
    assert found("0.001 of 0 and 350 of 11.7 and one of the two and 240 mm") == []


def test_a_bare_triple_followed_by_punctuation_is_still_a_claim_and_a_matching_report_backs_it(tree):
    assert [c for c, k in ceb.find_claims("25 / 3 / 0, each; 14 / 2 / 0. And 1,795 / 957 / 1,832")] == ["25 / 3 / 0", "14 / 2 / 0", "1,795 / 957 / 1,832"]
    assert statuses(check(tree, "Rows: 2 / 1 / 0, each.\n")) == ["BACKED"]
    assert statuses(check(tree, "Rows: 7 / 7 / 7, each.\n")) == ["UNBACKED!"]


def test_fenced_code_and_html_comments_are_not_scanned(tree):
    body = "```\n9 tests\n```\n<!-- 8 tests -->\ntext <!-- 7 tests --> after\nplain\n"
    assert statuses(check(tree, body)) == []


def test_a_verdict_count_that_a_committed_report_has_is_backed(tree):
    result = check(tree, "A7 gave 2 PASS / 1 WARN / 0 FAIL.\n")
    assert statuses(result) == ["BACKED"] and result["status"] == "PASS"
    assert result["claims"][0]["evidence"][0].startswith("evidence/reports/2026")


def test_a_verdict_count_no_report_has_fails(tree):
    result = check(tree, "A7 gave 9 PASS / 0 WARN / 0 FAIL.\n")
    assert statuses(result) == ["UNBACKED!"] and result["status"] == "FAIL"


def test_a_number_nothing_backs_fails_and_acknowledging_it_prints_it_without_failing(tree):
    body = "The bridge dies in about 1 in 7 runs.\n"
    assert check(tree, body)["status"] == "FAIL"
    ack = {"doc": "docs/pr/BODY.md", "claim": "about 1 in 7", "kind": "unbacked", "reason": "quoted so it can be retracted"}
    result = check(tree, body, [ack])
    assert statuses(result) == ["UNBACKED"] and result["status"] == "PASS"
    assert "quoted so it can be retracted" in result["claims"][0]["detail"]


def test_json_count_backs_a_rate_only_when_both_numbers_are_right(tree):
    entry = {"doc": "docs/pr/BODY.md", "claim": "1 of 2", "kind": "json_count",
             "numerator": {"glob": "evidence/reports/*_a7.json", "where": [{"any_check_result": "FAIL"}]},
             "denominator": {"glob": "evidence/reports/*_a7.json"}}
    assert statuses(check(tree, "1 of 2 runs failed.\n", [entry])) == ["BACKED"]
    assert statuses(check(tree, "2 of 2 runs failed.\n", [{**entry, "claim": "2 of 2"}])) == ["MISMATCH"]
    wrong_den = {**entry, "denominator": {"glob": "evidence/reports/*_a7.json", "where": [{"path": "git_hash", "startswith": "bbb"}]}}
    assert statuses(check(tree, "1 of 2 runs failed.\n", [wrong_den])) == ["MISMATCH"]


def test_where_conditions(tree):
    spec = lambda *w: {"glob": "evidence/reports/*_a7.json", "where": list(w)}
    assert ceb.count_reports(str(tree), spec({"path": "upstream.openamr-platform-sw", "equals": "abc1234"}))[0] == 2
    assert ceb.count_reports(str(tree), spec({"path": "git_hash", "startswith": "bbb"}))[0] == 1
    assert ceb.count_reports(str(tree), spec({"verdicts": {"PASS": 2, "WARN": 1, "FAIL": 0}}))[0] == 1
    assert ceb.count_reports(str(tree), spec({"check": "C1", "result": "FAIL"}))[0] == 1
    assert ceb.count_reports(str(tree), spec({"no_check_result": "FAIL"}))[0] == 1
    assert ceb.count_reports(str(tree), {"globs": ["evidence/reports/*_a7.json", "evidence/reports/*_dev.json"]})[0] == 3


def test_json_all_and_json_value(tree):
    all_entry = {"doc": "docs/pr/BODY.md", "claim": "0 excursions", "kind": "json_all", "glob": "evidence/reports/*_a7.json",
                 "path": "upstream.openamr-platform-sw", "equals": "abc1234", "at_least": 2}
    assert statuses(check(tree, "0 excursions in all.\n", [all_entry])) == ["BACKED"]
    assert statuses(check(tree, "0 excursions in all.\n", [{**all_entry, "at_least": 3}])) == ["MISMATCH"]
    assert statuses(check(tree, "0 excursions in all.\n", [{**all_entry, "equals": "zzz"}])) == ["MISMATCH"]
    value = {"doc": "docs/pr/BODY.md", "claim": "3 checks", "kind": "json_value", "file": "evidence/reports/20260101T000000Z_a7.json",
             "path": "checks", "length": True}
    assert statuses(check(tree, "It has 3 checks.\n", [value])) == ["BACKED"]
    assert statuses(check(tree, "It has 4 checks.\n", [{**value, "claim": "4 checks"}])) == ["MISMATCH"]
    by_id = {"doc": "docs/pr/BODY.md", "claim": "1 pairs", "kind": "json_value", "file": "evidence/reports/20260101T000000Z_a7.json",
             "path": "checks[C1].result"}
    assert ceb.dotted(json.loads((tree / "evidence/reports/20260101T000000Z_a7.json").read_text()), "checks[C1].result") == "PASS"
    assert ceb.dotted({"checks": []}, "checks[C1].result") is None and by_id["kind"] == "json_value"


def test_a_record_must_be_a_file_under_evidence_that_contains_the_text(tree):
    base = {"doc": "docs/pr/BODY.md", "claim": "2 of 8", "kind": "record", "file": "evidence/reports/README.md", "contains": "two of the 8 runs"}
    assert statuses(check(tree, "It was 2 of 8.\n", [base])) == ["RECORDED"]
    assert statuses(check(tree, "It was 2 of 8.\n", [{**base, "contains": "three of the 8"}])) == ["MISMATCH"]
    assert statuses(check(tree, "It was 2 of 8.\n", [{**base, "file": "docs/pr/BODY.md"}])) == ["MISMATCH"]
    assert statuses(check(tree, "It was 2 of 8.\n", [{**base, "file": "evidence/../docs/pr/BODY.md"}])) == ["MISMATCH"]
    assert statuses(check(tree, "It was 2 of 8.\n", [{**base, "file": "evidence/nothing.md"}])) == ["MISMATCH"]


def test_near_limits_an_entry_to_one_place(tree):
    body = "First: 3 runs here.\nSecond: 3 runs there.\n"
    one = {"doc": "docs/pr/BODY.md", "claim": "3 runs", "kind": "unbacked", "reason": "r", "near": "second:"}
    result = check(tree, body, [one])
    assert statuses(result) == ["UNBACKED", "UNBACKED!"] and result["status"] == "FAIL"


def test_a_ledger_entry_that_matches_nothing_fails(tree):
    stale = {"doc": "docs/pr/BODY.md", "claim": "7 pairs", "kind": "unbacked", "reason": "gone"}
    result = check(tree, "Nothing to count here.\n", [stale])
    assert statuses(result) == ["UNUSED"] and result["status"] == "FAIL"


def test_a_table_row_must_agree_with_the_reports_it_names(tree):
    index = tree / "evidence" / "reports" / "README.md"
    index.write_text("| file | result |\n|---|---|\n| `20260101T000000Z_a7.json`, `...T000000Z_dev.json` | 3 PASS, 1 WARN, 0 FAIL |\n")
    result = check(tree, "No numbers.\n")
    assert statuses(result) == ["MISMATCH"] and "20260101T000000Z_a7.json has 2 PASS" in result["claims"][0]["detail"]
    index.write_text("| file | result |\n|---|---|\n| `20260101T000000Z_a7.json` | 2 PASS, 1 WARN, 0 FAIL |\n")
    assert statuses(check(tree, "No numbers.\n")) == ["BACKED"]
    index.write_text("| file | result |\n|---|---|\n| `20260109T000000Z_a7.json` | 2 PASS, 1 WARN, 0 FAIL |\n")
    assert statuses(check(tree, "No numbers.\n")) == ["MISMATCH"]


def test_numbers_in_a_generated_reports_prose_are_scanned_and_keyed_by_profile(tree):
    planted = report(extra={"limitations": ["the bridge exits -11 in 3 of 5 launches on this machine"]})
    (tree / "evidence" / "reports" / "20260104T000000Z_a7.json").write_text(json.dumps(planted))
    result = check(tree, "No numbers.\n", reports=["evidence/reports/20260104T000000Z_a7.json"])
    assert statuses(result) == ["UNBACKED!"] and result["claims"][0]["doc"] == "report:a7"
    ack = {"doc": "report:a7", "claim": "3 of 5", "kind": "unbacked", "reason": "r"}
    assert check(tree, "No numbers.\n", [ack], reports=["evidence/reports/20260104T000000Z_a7.json"])["status"] == "PASS"


def test_an_observed_but_unrecorded_rate_must_say_so_in_the_text(tree):
    entry = {"doc": "docs/pr/BODY.md", "claim": "2 of 9", "kind": "observed_not_recorded", "observed_by": "console"}
    bare = check(tree, "The exit was seen in 2 of 9 launches.\n", [entry])
    assert statuses(bare) == ["MISMATCH"] and bare["status"] == "FAIL"
    labelled = check(tree, "The exit was seen (OBSERVED-NOT-RECORDED) in 2 of 9 launches.\n", [entry])
    assert statuses(labelled) == ["OBSERVED-NOT-RECORDED"] and labelled["status"] == "PASS"
    assert check(tree, "The exit was seen in 2 of 9 launches.\n")["status"] == "FAIL"


def test_launch_exits_derive_a_rate_from_recorded_exit_codes_and_only_from_them(tree):
    def launch(code):
        return {"check": "G02", "sidecar": "G02_x.json", "processes": [{"process": "parameter_bridge-4", "exit_code": code}, {"process": "gazebo-1", "exit_code": -2}]}
    data = report(extra={"process_exits": [launch(0), launch(-11), launch(0), launch(-6)]})
    (tree / "evidence" / "reports" / "20260105T000000Z_exits.json").write_text(json.dumps(data))
    entry = {"doc": "docs/pr/BODY.md", "claim": "1 of 4", "kind": "launch_exits", "glob": "evidence/reports/*_exits.json",
             "process": "parameter_bridge", "exit_code": -11}
    assert ceb.count_launches(str(tree), entry) == (1, 4, ["evidence/reports/20260105T000000Z_exits.json"])
    assert statuses(check(tree, "A bridge ended in -11 in 1 of 4 launches.\n", [entry])) == ["BACKED"]
    assert statuses(check(tree, "A bridge ended in -11 in 2 of 4 launches.\n", [{**entry, "claim": "2 of 4"}])) == ["MISMATCH"]
    assert statuses(check(tree, "A bridge ended in -6 in 1 of 4 launches.\n", [{**entry, "exit_code": -6}])) == ["BACKED"]
    # the reports of the other fixtures have no process_exits: they contribute no launch
    assert ceb.count_launches(str(tree), {"glob": "evidence/reports/2026010[123]*_a7.json", "process": "parameter_bridge", "exit_code": -11})[:2] == (0, 0)


def test_the_cli_and_its_exit_codes(tree, capsys):
    (tree / "docs" / "pr" / "BODY.md").write_text("It was 3 of 5.\n")
    (tree / "ledger.yaml").write_text(yaml.safe_dump({"claims": []}))
    out = tree / "out.json"
    argv = ["--repo-root", str(tree), "--ledger", str(tree / "ledger.yaml"), "--doc", "docs/pr/BODY.md", "--no-default-reports", "--json", str(out)]
    assert ceb.main(argv) == 1 and json.loads(out.read_text())["status"] == "FAIL"
    assert "UNBACKED" in capsys.readouterr().out
    assert ceb.main(argv + ["--skeleton"]) == 0 and "kind: unbacked" in capsys.readouterr().out
    assert ceb.main(["--repo-root", str(tree), "--ledger", str(tree / "nope.yaml"), "--doc", "docs/pr/BODY.md"]) == 2
    assert ceb.main(["--repo-root", str(tree), "--ledger", str(tree / "ledger.yaml"), "--doc", "docs/pr/NOPE.md"]) == 2


def test_the_canary_catches_every_planted_number(tree):
    (tree / "docs" / "pr" / "PR14_BODY.md").write_text("Nothing countable.\n")
    (tree / "ledger.yaml").write_text(yaml.safe_dump({"claims": []}))
    ok, lines = ceb.canary(str(tree), str(tree / "ledger.yaml"))
    assert ok, "\n".join(lines)
    assert sum(line.startswith("CAUGHT") for line in lines) == 12 and "PASS" in lines[0]


def test_the_canary_fails_when_the_check_is_blind(tree, monkeypatch):
    (tree / "docs" / "pr" / "PR14_BODY.md").write_text("Nothing countable.\n")
    (tree / "ledger.yaml").write_text(yaml.safe_dump({"claims": []}))
    monkeypatch.setattr(ceb, "PATTERNS", [p for p in ceb.PATTERNS if p[0] not in ("rate_in", "rate_of")])      # a check that cannot see a rate
    ok, lines = ceb.canary(str(tree), str(tree / "ledger.yaml"))
    assert not ok and any(line.startswith("MISSED") for line in lines)
