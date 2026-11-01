# SPDX-License-Identifier: MIT
"""scripts/build_evidence.py: the report only combines what the tests recorded. Its mapping and combining rules are
tested here, on synthetic results, including that a FAIL cannot be hidden by a PASS and that a missing result is not a pass.
"""
import json
import os
import subprocess
import sys
import time

import pytest

import build_evidence as be
import evidence_common as ev
from conftest import SCRIPTS


def test_every_a6_check_id_in_verification_md_is_in_the_report():
    ids = [i for i, _ in be.A6_CHECKS]
    assert ids == [f"T{n:02d}" for n in range(1, 17)] + ["L01", "E01"]
    assert len(set(i for i, _ in be.A6_CHECKS + be.EXTRA_CHECKS)) == len(be.A6_CHECKS + be.EXTRA_CHECKS)
    assert set(be.NOT_IMPLEMENTED) <= set(ids)
    assert be.SIDECAR_REQUIRED <= {i for i, _ in be.A6_CHECKS + be.EXTRA_CHECKS + be.GAZEBO_CHECKS}


@pytest.mark.parametrize("classname,name,expected", [
    ("pkg.test.test_t05_shoulder_height", "test_t05_shoulder_height_matches_the_lift_stop[L1000]", "T05"),
    ("pkg.test.test_t07_collision", "test_t08_collision_mesh_same_file_as_visual_mesh_is_a_warning", "T08"),   # function wins
    ("pkg.test.test_t04_frame_contract", "test_checker_fails_a_missing_r_frame", "T04"),                      # module fallback
    ("pkg.test.test_w01_kdl_root_inertia", "test_w01_base_link_inertia_is_seen_by_kdl_consumers", "W01"),
    ("pkg.test.test_t04_optb_composition", "test_the_numbers_option_b_put_on_main_are_the_ones_the_model_uses", "T04"),   # module fallback
    ("pkg.test.test_e01_envelope", "test_e01_assembled_height_against_the_envelope", "E01"),
    ("pkg.test.test_c01_config_id", "test_c01_command_prints_exactly_the_full_id", "C01"),
    ("pkg.test.test_c02_upstream_pin", "test_c02_a_wrong_pin_fails_and_names_both_hashes", "C02"),
    ("pkg.test.test_datums_revision", "test_guard_skips_with_a_single_commit", "REV"),
    ("pkg.test.test_datums_report", "test_report_exits_zero_and_lists_every_placeholder", "AUX"),
    ("pkg.test.test_s4_sensors", "test_camera_links_are_visual_only[head]", "AUX"),
    ("pkg.test.test_urdf_utils", "test_rpy_axes_follow_the_urdf_convention", "AUX"),
    ("pkg.test.test_checker_like", "test_checker_without_id", "AUX"),
    ("pkg.test.test_g01_gazebo_smoke", "test_g01_3_the_nav2_facing_topics_are_platform_sws_by_name_and_type", "G01"),
    ("pkg.test.test_g02_gripper_mimic", "test_g02_1_both_grippers_close_and_the_mimic_follows", "G02"),
])
def test_check_id_mapping(classname, name, expected):
    assert be.check_id_of(classname, name) == expected


GOOD = {"tests": 5, "failures": 0, "skipped": 0, "failed_tests": []}


def _sidecar(result, **details):
    return {"id": "X", "name": "from sidecar", "result": result, "details": details}


@pytest.mark.parametrize("check_id,counts,sidecar,expected", [
    ("T01", GOOD, None, "PASS"),
    ("T01", {**GOOD, "failures": 1, "failed_tests": ["t"]}, None, "FAIL"),
    ("T04", GOOD, _sidecar("PASS"), "PASS"),
    ("T04", GOOD, _sidecar("WARN"), "WARN"),
    ("T04", {**GOOD, "failures": 1}, _sidecar("PASS"), "FAIL"),            # a PASS sidecar cannot hide a failing test
    ("T04", GOOD, _sidecar("FAIL"), "FAIL"),                               # nor a passing test hide a FAIL sidecar
    ("T04", {**GOOD, "failures": 1}, _sidecar("WARN"), "FAIL"),            # nor a WARN
    ("T10", GOOD, None, "NOT_RUN"),                                        # sidecar required, test did not complete
    ("E01", None, None, "NOT_RUN"),
    ("T05", None, None, "NOT_RUN"),                                        # no result is not a pass
    ("T14", None, None, "NOT_RUN"),                                        # implemented since P06: no result is NOT_RUN
    ("L01", None, None, "NOT_RUN"),
    ("T01", {**GOOD, "skipped": 5}, None, "SKIP"),                         # every test skipped
    ("T01", {**GOOD, "skipped": 2}, None, "PASS"),                         # some ran and passed
    ("C01", {**GOOD, "skipped": 2}, _sidecar("PASS"), "PASS"),
    ("C02", GOOD, _sidecar("PASS"), "PASS"),
    ("C02", {**GOOD, "skipped": 1}, _sidecar("SKIP", reason="no checkout"), "SKIP"),   # the one legitimate SKIP, by sidecar
    ("C02", GOOD, _sidecar("FAIL"), "FAIL"),                                           # drift is never softened
    ("C02", GOOD, None, "NOT_RUN"),                                                    # the recording test did not complete
    ("T04", None, _sidecar("PASS"), "NOT_RUN"),                                        # a sidecar with no test result behind it is not a pass (P12A)
    ("T04", None, _sidecar("WARN"), "NOT_RUN"),                                        # nor a WARN
    ("C01", {**GOOD, "skipped": 5}, _sidecar("PASS"), "SKIP"),                          # every test skipped: nothing ran behind the sidecar
])
def test_combine(check_id, counts, sidecar, expected):
    assert be.combine(check_id, counts, sidecar)[0] == expected


@pytest.mark.parametrize("counts,sidecar,missing,expected", [
    (GOOD, None, ["t_b"], "NOT_RUN"),                                        # all that ran passed, but a registered test left no result
    (GOOD, _sidecar("PASS"), ["t_b"], "NOT_RUN"),                            # a sidecar does not stand in for it
    ({**GOOD, "failures": 1, "failed_tests": ["t"]}, None, ["t_b"], "FAIL"),  # a failure outranks a missing result
    (None, None, ["t_b"], "NOT_RUN"),
    (GOOD, _sidecar("FAIL"), ["t_b"], "FAIL"),
])
def test_a_registered_test_with_no_result_makes_its_check_not_run_and_never_a_pass(counts, sidecar, missing, expected):
    result, details = be.combine("T01", counts, sidecar, missing)
    assert result == expected
    assert details["tests_without_a_result"] == missing


def test_not_run_is_a_value_of_its_own_and_fails_the_exit_status():
    assert be.NOT_RUN == "NOT_RUN" and be.NOT_RUN not in ("PASS", "WARN", "SKIP", "FAIL")
    assert be.NOT_RUN in be.FAILING and "FAIL" in be.FAILING and "SKIP" not in be.FAILING
    assert be.NOT_RUN in be.RESULT_ORDER


@pytest.mark.parametrize("name,expected", [
    ("t04_frame_contract", "T04"), ("test_test_l01_mock_controllers.py", "L01"), ("m01_mock_named_poses", "M01"), ("w01_kdl_root_inertia", "W01"),
    ("s4_sensors", "AUX"), ("datums_revision", "REV"), ("canary_pytest_assert", "AUX"), ("joint_state_clamp_logic", "AUX"),
])
def test_a_registered_test_is_attributed_to_a_check_by_its_name(name, expected):
    assert be.test_name_id(name) == expected


def test_results_filed_under_an_id_the_report_does_not_list_are_folded_into_aux_not_dropped():
    """A test named `test_q99_...` maps to check Q99, which no report lists: before P12A its failure was read by nobody."""
    counts = {"T01": dict(GOOD), "Q99": {"tests": 1, "failures": 1, "skipped": 0, "failed_tests": ["test_q99_x"]},
              "M01": {"tests": 2, "failures": 0, "skipped": 2, "failed_tests": []}}
    unclaimed = be.fold_unclaimed(counts, {"T01", "AUX"})
    assert set(unclaimed) == {"Q99", "M01"} and "Q99" not in counts
    assert counts["AUX"] == {"tests": 3, "failures": 1, "skipped": 2, "failed_tests": ["test_q99_x"]}
    assert be.combine("AUX", counts["AUX"], None)[0] == "FAIL"
    assert be.missing_by_id([{"id": "Q99", "test": "q99_x", "package": "p"}, {"id": "T01", "test": "t01_a", "package": "p"}], {"T01", "AUX"}) == {
        "AUX": ["q99_x"], "T01": ["t01_a"]}


def _results(tmp_path, tests):
    """A fake workspace: build/<pkg>/test_results/<pkg>/<test>.xunit.xml for {test: inner xml of a testcase or None for no file}."""
    d = tmp_path / "build" / "pkg" / "test_results" / "pkg"
    d.mkdir(parents=True, exist_ok=True)
    for test, inner in tests.items():
        if inner is not None:
            name = f"{test}.xunit.missing_result" if inner == "MISSING" else "test_x"
            body = "<error message='no result'/>" if inner == "MISSING" else inner
            (d / f"{test}.xunit.xml").write_text(f'<testsuite><testcase classname="p.test" name="{name}">{body}</testcase></testsuite>')
    return str(d)


@pytest.mark.parametrize("inner,status", [("", "ok"), ("<failure message='x'/>", "ok"), ("MISSING", "did_not_complete"), (None, "no_result_file")])
def test_result_status_tells_a_missing_a_stand_in_and_a_real_result_apart(tmp_path, inner, status):
    d = _results(tmp_path, {"t01_a": inner})
    assert be.result_status(d, "t01_a", since=time.time() - 3600) == status


def test_result_status_stale_empty_and_unreadable(tmp_path):
    d = _results(tmp_path, {"t01_a": ""})
    os.utime(os.path.join(d, "t01_a.xunit.xml"), (1, 1))
    assert be.result_status(d, "t01_a", since=time.time() - 3600) == "stale"
    open(os.path.join(d, "t01_b.xunit.xml"), "w").write("<testsuite/>")
    assert be.result_status(d, "t01_b", since=0) == "empty"
    open(os.path.join(d, "t01_c.xunit.xml"), "w").write("not xml")
    assert be.result_status(d, "t01_c", since=0) == "unreadable"


def test_accounting_lists_registered_tests_without_a_result_and_a_package_with_none_registered(tmp_path, monkeypatch):
    d = _results(tmp_path, {"t01_a": "", "t02_b": None, "canary_timeout": "MISSING"})
    monkeypatch.setattr(be, "registered_tests", lambda ws, pkg: {"pkg": ["t01_a", "t02_b", "canary_timeout"], "empty": []}[pkg])
    got = be.accounting(str(tmp_path), ["pkg", "empty"], since=time.time() - 3600)
    assert got["registered"] == 3 and got["with_a_result"] == 1
    assert {(p["package"], p["test"], p["status"], p["id"]) for p in got["problems"] if p["test"]} == {
        ("pkg", "t02_b", "no_result_file", "T02"), ("pkg", "canary_timeout", "did_not_complete", "AUX")}
    empty, = [p for p in got["problems"] if p["package"] == "empty"]
    assert empty["test"] is None and "no registered tests" in empty["status"], "a suite that vanished must not look like a suite that passed"


def test_a_package_whose_ctest_project_cannot_be_read_is_a_problem_not_a_pass(tmp_path):
    assert be.registered_tests(str(tmp_path), "nonexistent_package") is None
    got = be.accounting(str(tmp_path), ["nonexistent_package"], since=0)
    assert got["problems"] and got["registered"] == 0


def test_a_check_listed_as_not_implemented_is_a_skip_with_the_reason(monkeypatch):
    monkeypatch.setitem(be.NOT_IMPLEMENTED, "T14", "no test yet")
    assert be.combine("T14", None, None) == ("SKIP", {"reason": "no test yet"})


def test_combine_keeps_the_sidecar_details_and_the_test_counts():
    result, details = be.combine("T09", {**GOOD, "failures": 0}, _sidecar("PASS", massed_links=36))
    assert result == "PASS" and details["massed_links"] == 36 and details["tests"] == 5


def test_trees_differ_reports_changed_added_and_missing_files(tmp_path):
    for side in ("src", "inst"):
        (tmp_path / side / "urdf").mkdir(parents=True)
        (tmp_path / side / "urdf" / "a.xacro").write_text("same")
        (tmp_path / side / "config").mkdir()
        (tmp_path / side / "config" / "d.yaml").write_text("same")
    assert be.trees_differ(str(tmp_path / "src"), str(tmp_path / "inst")) == []
    (tmp_path / "src" / "urdf" / "a.xacro").write_text("changed")
    (tmp_path / "src" / "urdf" / "new.xacro").write_text("only in src")
    (tmp_path / "inst" / "config" / "old.yaml").write_text("only installed")
    assert be.trees_differ(str(tmp_path / "src"), str(tmp_path / "inst")) == [
        "config/old.yaml", "urdf/a.xacro", "urdf/new.xacro"]


def test_trees_differ_ignores_content_free_leftovers_but_not_a_stale_installed_xacro(tmp_path):
    for side in ("src", "inst"):
        (tmp_path / side / "urdf").mkdir(parents=True)
        (tmp_path / side / "urdf" / "a.xacro").write_text("same")
    (tmp_path / "inst" / "urdf" / ".gitkeep").symlink_to(tmp_path / "gone")          # dangling, as left by an old scaffold
    (tmp_path / "inst" / "urdf" / "__pycache__").mkdir()
    assert be.trees_differ(str(tmp_path / "src"), str(tmp_path / "inst")) == []
    (tmp_path / "inst" / "urdf" / "stale.xacro").write_text("removed from the source long ago")
    assert be.trees_differ(str(tmp_path / "src"), str(tmp_path / "inst")) == ["urdf/stale.xacro"]


def _xunit(path, cases):
    body = "".join(
        f'<testcase classname="p.test.{mod}" name="{name}">{inner}</testcase>' for mod, name, inner in cases)
    path.write_text(f'<?xml version="1.0"?><testsuites><testsuite name="pytest">{body}</testsuite></testsuites>')


def test_read_xunit_counts_per_id_and_ignores_stale_files(tmp_path):
    _xunit(tmp_path / "a.xunit.xml", [
        ("test_t01_expansion", "test_t01_ok[1]", ""), ("test_t01_expansion", "test_t01_bad", "<failure message='x'/>"),
        ("test_t01_expansion", "test_t01_err", "<error message='x'/>"), ("test_t01_expansion", "test_t01_skip", "<skipped/>")])
    _xunit(tmp_path / "old.xunit.xml", [("test_t03_tree", "test_t03_x", "")])
    os.utime(tmp_path / "old.xunit.xml", (1, 1))
    counts, stale = be.read_xunit(str(tmp_path), since=time.time() - 3600)
    assert counts["T01"] == {"tests": 4, "failures": 2, "skipped": 1, "failed_tests": ["test_t01_bad", "test_t01_err"]}
    assert "T03" not in counts and stale == ["old.xunit.xml"]


def test_read_xunit_dirs_merges_a_second_package_and_adds_counts_for_one_id(tmp_path):
    (tmp_path / "a").mkdir()
    (tmp_path / "b").mkdir()
    _xunit(tmp_path / "a" / "a.xunit.xml", [("test_t01_expansion", "test_t01_ok", "")])
    _xunit(tmp_path / "b" / "launch.xunit.xml", [("TestMockControllers", "test_l01_1_all_five_controllers_active", ""),
                                                 ("TestShutdown", "test_l01_processes_exit_cleanly", "<failure message='x'/>")])
    counts, stale = be.read_xunit_dirs([str(tmp_path / "a"), str(tmp_path / "b"), str(tmp_path / "missing")],
                                       since=time.time() - 3600)
    assert counts["T01"]["tests"] == 1 and stale == []
    assert counts["L01"]["tests"] == 2 and counts["L01"]["failures"] == 1
    assert be.combine("L01", counts["L01"], None)[0] == "FAIL"        # one failing L01 test fails L01


def test_a_stale_sidecar_is_not_used(tmp_path):
    ev.write_sidecar("T99", "probe", "probe", "PASS", {}, directory=str(tmp_path))
    old = next(p for p in tmp_path.iterdir())
    os.utime(old, (1, 1))
    ev.write_sidecar("T98", "fresh", "fresh", "WARN", {}, directory=str(tmp_path))
    assert set(be.fresh_sidecars(str(tmp_path), since=time.time() - 3600)) == {"T98"}


def _repo(tmp_path, dirty=False):
    """A throwaway git repository with one commit, standing in for openamr-upperbody-sw (--repo)."""
    repo = tmp_path / "repo"
    repo.mkdir()
    git = lambda *a: subprocess.run(["git", "-C", str(repo), *a], check=True, capture_output=True)  # noqa: E731
    git("init", "-q")
    (repo / "tracked.txt").write_text("a")
    git("add", "tracked.txt")
    git("-c", "user.name=t", "-c", "user.email=t@t", "commit", "-q", "-m", "x")
    if dirty:
        (repo / "tracked.txt").write_text("changed")
        (repo / "untracked.txt").write_text("new")
    return str(repo)


def _run_build(tmp_path, repo):
    evidence = tmp_path / "evidence"
    result = subprocess.run([sys.executable, os.path.join(SCRIPTS, "build_evidence.py"), "--since", str(time.time()),
                             "--results-dir", str(tmp_path / "none"), "--repo", repo], capture_output=True, text=True,
                            env={**os.environ, ev.EVIDENCE_ENV: str(evidence)})
    return result, [p for p in evidence.iterdir() if p.name.endswith("_dev.json")] if evidence.is_dir() else []


def test_summary_and_exit_status_follow_the_checks(tmp_path):
    # end to end on the real installed package with an empty results directory: nothing ran, so it must FAIL, and still write
    # (the repository it describes is a clean one: a dirty tree writes nothing, see below)
    result, reports = _run_build(tmp_path, _repo(tmp_path))
    assert result.returncode == 1, result.stdout + result.stderr
    assert len(reports) == 1
    doc = json.loads(reports[0].read_text())
    assert doc["schema"] == "openamr.evidence.v1" and doc["evidence_class"] == "static+mock" and doc["profile"] == "dev"
    assert doc["dirty"] is False and "dirty_files" not in doc
    by_id = {c["id"]: c["result"] for c in doc["checks"]}
    assert by_id["T01"] == "NOT_RUN" and by_id["T14"] == "NOT_RUN" and by_id["L01"] == "NOT_RUN" and by_id["V01"] == "WARN"
    v01 = next(c for c in doc["checks"] if c["id"] == "V01")
    assert [f["id"] for f in v01["details"]["findings"]] == ["V01-F1", "V01-F2"]            # the look found two defects: they are in the report
    assert any("V01-F1" in line for line in doc["limitations"])
    assert by_id["AUX"] == "FAIL"                                          # the three steps were never run
    assert by_id["H02"] == "NOT_RUN"                                       # and nothing was registered to account for: that is not a pass either
    assert set(doc["config_ids"]) and all(ev.SHA256_RE.match(v) for v in doc["config_ids"].values())
    assert len(doc["config_ids"]) == 14 and "dold_v4_350/bp000/hp25/bt10/openarm/mock" in doc["config_ids"]
    assert not [k for k in doc["config_ids"] if "mast" in k]
    # the inputs of the id and the calibration IDs are stated beside the ids, apart from them
    assert doc["config_id_inputs"]["members"] == ["canonical_urdf", "column", "base_plate", "head_pitch", "base_tilt", "arm_package",
                                                  "hardware_profile", "datums_revision", "upstream_pins"]
    assert "mast_id" in doc["config_id_inputs"]["before_p15"] and "mast_id" not in doc["config_id_inputs"]["members"]  # decision-allow: CONFIG-IDS negative test: mast_id must be absent or rejected
    assert set(doc["calibration_ids"]) == {"L1000", "L1175", "L1350"} and not set(doc["calibration_ids"]) & set(doc["config_ids"])
    assert [doc["calibration_ids"][s]["lift_joint_position_m"] for s in ("L1000", "L1175", "L1350")] == [0.0, 0.175, 0.35]
    assert doc["limitations"] and all(isinstance(x, str) for x in doc["limitations"])
    assert "Result: FAIL" in result.stdout and f"Evidence: {reports[0]}" in result.stdout and "NOT_RUN" in result.stdout


def test_v01_records_what_the_looking_found_and_stays_warn():
    """V01 was WARN with 'nobody has looked'. A person has now looked and found two defects no test was checking. The record must say
    so, and must not become a PASS: only item 1 has an answer, and a check whose value was finding defects is not 'found nothing'."""
    details = be.v01_details(8)
    assert [f["id"] for f in details["findings"]] == ["V01-F1", "V01-F2"]
    # P16 measured the eight items of the lift model: each has a verdict and the number read beside the number expected
    assert sorted(details["answered_items"], key=int) == ["1", "2", "3", "4", "5", "6", "7", "8"] and details["unanswered_items"] == []
    assert all(a["verdict"] in ("PASS", "FAIL", "CANNOT-MEASURE") and a["expected"] and a["measured"] for a in details["answered_items"].values())
    assert "lift_mass_tl3_400" in details["stays_warn_because"]
    assert set(details["answered_before_the_lift"]) == {"1"}                 # the mast's item 1, kept as history and not counted
    text = json.dumps(details)
    assert "probably rotated 90 degrees" in text and "implausible" in text
    assert "found what the automated ones could not" in details["reading"] and "V01 stays WARN" in details["reading"]
    assert "no human has" not in text and "nobody has" not in text           # the record no longer says the check found nothing
    f1, f2 = details["findings"]
    assert "confirmed correct by the design owner" in f1["outcome"] and "closed by the supplier catalogue figure" in f2["outcome"]
    assert "11.7 kg" in f2["outcome"] and "11.7 kg" in f2["resolved_by"]
    for finding in details["findings"]:
        assert finding["outcome"] and finding["resolved_by"]                 # a finding with no outcome and no resolution is a complaint
    details["findings"][0]["id"] = "changed"                                 # callers get copies: one report cannot edit the next
    assert be.v01_details(8)["findings"][0]["id"] == "V01-F1"


def _run_gazebo_build(tmp_path, repo, *steps):
    evidence = tmp_path / "evidence"
    result = subprocess.run([sys.executable, os.path.join(SCRIPTS, "build_evidence.py"), "--profile", "gazebo", "--since",
                             str(time.time()), "--gazebo-results-dir", str(tmp_path / "none"), "--repo", repo, *steps],
                            capture_output=True, text=True, env={**os.environ, ev.EVIDENCE_ENV: str(evidence)})
    return result, sorted(p for p in evidence.iterdir() if p.name.endswith("_gazebo.json")) if evidence.is_dir() else []


PHYSICAL_CLAIMS = ("works on hardware", "production-ready", "validated")


def assert_simulation_only_contract(limitations):
    """The contract of a `gazebo` report's limitations, as PRESENCE and not position: the simulation-only disclaimer appears (in any entry, once is
    enough) and no physical claim appears anywhere. Where the disclaimer sits is not a safety property: requiring it LAST made every new limitation
    (an upstream pin read from metadata, for one) break the test, which is what happened on a checkout without the upstream clones."""
    assert all(isinstance(x, str) for x in limitations)
    assert any("no physical evidence" in line and "simulation only" in line for line in limitations), "the simulation-only disclaimer is absent"
    text = " ".join(limitations)
    for claim in PHYSICAL_CLAIMS:
        assert claim not in text.replace("not validated", ""), f"a physical claim appears: {claim!r}"


DISCLAIMER = "no physical evidence; simulation only. This report is not a statement about hardware"


def test_a_limitation_appended_after_the_disclaimer_still_passes_the_contract():
    assert_simulation_only_contract(["something", DISCLAIMER, "upstream pin openarm_ros2 (4e837e1) was read from config/upstream_pins.yaml, not from a git checkout"])


def test_the_disclaimer_in_any_position_passes_and_a_report_without_it_fails():
    assert_simulation_only_contract([DISCLAIMER])
    assert_simulation_only_contract([DISCLAIMER, "b", "c"])
    assert_simulation_only_contract(["a", DISCLAIMER, "c"])
    for missing in ([], ["only other text"], ["no physical evidence"], ["simulation only"], ["no physical evidence; a different sentence"]):
        with pytest.raises(AssertionError, match="disclaimer is absent"):
            assert_simulation_only_contract(missing)


@pytest.mark.parametrize("claim", PHYSICAL_CLAIMS)
def test_a_physical_claim_anywhere_fails_the_contract_even_with_the_disclaimer_present(claim):
    for lines in ([f"the arm {claim}", DISCLAIMER], [DISCLAIMER, f"the arm {claim}"]):
        with pytest.raises(AssertionError, match="physical claim"):
            assert_simulation_only_contract(lines)


def test_not_validated_is_not_a_physical_claim():
    assert_simulation_only_contract(["this is not validated", DISCLAIMER])


def test_the_real_gazebo_limitations_satisfy_the_contract_with_and_without_a_later_note():
    """The generator's own text, against the contract: its disclaimer is present, and a later limitation does not break it."""
    sims = {"gz_sim": "8", "gz_ros2_control": "1", "ros_gz_sim": "1"}
    lines = be.gazebo_limitations({}, {}, sims)
    assert_simulation_only_contract(lines)
    assert_simulation_only_contract(lines + ["upstream pin openarm_description (14ff67b) was read from config/upstream_pins.yaml, not from a git checkout"])


def test_the_gazebo_profile_writes_a_simulation_report_and_never_a_physical_claim(tmp_path):
    result, reports = _run_gazebo_build(tmp_path, _repo(tmp_path))
    assert len(reports) == 1, result.stdout + result.stderr
    doc = json.loads(reports[0].read_text())
    assert doc["profile"] == "gazebo" and doc["evidence_class"] == "simulation" and doc["dirty"] is False
    assert [c["id"] for c in doc["checks"]] == [i for i, _ in be.GAZEBO_CHECKS]
    by_id = {c["id"]: c["result"] for c in doc["checks"]}
    assert by_id["G01"] == "NOT_RUN" and by_id["G02"] == "NOT_RUN"                 # nothing ran: no result is not a pass
    v01 = next(c for c in doc["checks"] if c["id"] == "V01")
    assert by_id["V01"] == "WARN" and [f["id"] for f in v01["details"]["findings"]] == ["V01-F1", "V01-F2"]
    assert set(doc["config_ids"]) == {"dold_v4_350/bp000/hp25/bt10/openarm/gazebo", "dold_v4_350/bp050/hp25/bt10/openarm/gazebo"}
    assert set(doc["simulator"]) == {"gz_sim", "gz_ros2_control", "ros_gz_sim"}
    assert_simulation_only_contract(doc["limitations"])


def test_the_gazebo_profile_refuses_a_dirty_tree_like_the_dev_one(tmp_path):
    result, reports = _run_gazebo_build(tmp_path, _repo(tmp_path, dirty=True))
    assert reports == [] and "Evidence: NOT WRITTEN" in result.stdout


def test_a_gazebo_check_with_no_sidecar_is_not_run_not_a_pass():
    assert {"G01", "G02"} <= be.SIDECAR_REQUIRED
    assert be.combine("G01", GOOD, None)[0] == "NOT_RUN"
    assert be.combine("G01", GOOD, _sidecar("PASS"))[0] == "PASS"
    assert be.combine("G02", GOOD, _sidecar("FAIL"))[0] == "FAIL"


def test_a_dirty_tree_writes_no_evidence_file_and_says_why(tmp_path):
    """The P08 failure: a dirty-tree run was left beside the clean one and the reviewer had to be told which to trust."""
    result, reports = _run_build(tmp_path, _repo(tmp_path, dirty=True))
    assert reports == [], "a report was written from a dirty tree"
    assert "Evidence: NOT WRITTEN" in result.stdout and "uncommitted changes" in result.stdout
    assert "tracked.txt" in result.stdout and "untracked.txt" in result.stdout        # names the files, tracked or not
    assert "(dirty)" in result.stdout                                                  # the console results still print
    assert result.returncode == 1                                                      # the FAILs of this empty run win


def test_a_dirty_tree_with_no_failing_check_exits_3_not_0(monkeypatch, capsys, tmp_path):
    """Exit 0 would read as 'recorded'. Nothing was, so it is its own status."""
    doc = {"repo": "openamr-upperbody-sw", "profile": "dev", "git_hash": "a" * 40, "dirty": True, "dirty_files": ["x.py"],
           "datums_revision": "r", "evidence_class": "static+mock", "config_ids": {}, "checks": [
               {"id": "T01", "name": "n", "result": "PASS", "details": {}}]}
    monkeypatch.setattr(be, "build", lambda args: (doc, None, {}))
    assert be.main(["--since", "0"]) == be.REFUSED_DIRTY == 3
    assert "NOT WRITTEN" in capsys.readouterr().out
    monkeypatch.setattr(be, "build", lambda args: ({**doc, "dirty": False, "dirty_files": []}, str(tmp_path / "r.json"), {}))
    assert be.main(["--since", "0"]) == 0


def test_clear_mode_removes_only_sidecars(tmp_path):
    ev.write_sidecar("T99", "probe", "probe", "PASS", {}, directory=str(tmp_path))
    (tmp_path / "S4.json").write_text("{}")
    result = subprocess.run([sys.executable, os.path.join(SCRIPTS, "build_evidence.py"), "--clear"], capture_output=True,
                            text=True, env={**os.environ, ev.EVIDENCE_ENV: str(tmp_path)})
    assert result.returncode == 0 and result.stdout.strip() == str(tmp_path)
    assert sorted(os.listdir(tmp_path)) == ["S4.json"]


def test_each_profile_clears_only_its_own_sidecars(tmp_path):
    """verify_gazebo.sh must not wipe the A6 sidecars the dev report was built from, nor the other way round."""
    for check_id in ("T99", "G01"):
        ev.write_sidecar(check_id, "probe", "probe", "PASS", {}, directory=str(tmp_path))
    env = {**os.environ, ev.EVIDENCE_ENV: str(tmp_path)}
    run = lambda *a: subprocess.run([sys.executable, os.path.join(SCRIPTS, "build_evidence.py"), *a, "--clear"],  # noqa: E731
                                    capture_output=True, text=True, env=env)
    assert run("--profile", "gazebo").returncode == 0 and sorted(os.listdir(tmp_path)) == ["T99_probe.json"]
    ev.write_sidecar("G01", "probe", "probe", "PASS", {}, directory=str(tmp_path))
    assert run("--profile", "dev").returncode == 0 and sorted(os.listdir(tmp_path)) == ["G01_probe.json"]


def test_the_gazebo_aux_check_is_decided_by_its_step_alone(tmp_path):
    """AUX has no test of its own in this profile: a step that ran and passed is PASS, a step that failed or never ran is FAIL."""
    def aux(*steps, name):
        result, reports = _run_gazebo_build(tmp_path / name, _repo(tmp_path / name), *steps)
        return next(c for c in json.loads(reports[0].read_text())["checks"] if c["id"] == "AUX")
    (tmp_path / "ran").mkdir(), (tmp_path / "failed").mkdir(), (tmp_path / "missing").mkdir()
    assert aux("--step", "colcon_test=0", name="ran")["result"] == "PASS"
    assert aux("--step", "colcon_test=1", name="failed")["result"] == "FAIL"
    assert aux(name="missing")["result"] == "FAIL"
