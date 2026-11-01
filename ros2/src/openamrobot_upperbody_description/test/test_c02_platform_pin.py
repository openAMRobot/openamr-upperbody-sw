# SPDX-License-Identifier: MIT
"""C02 must compare the platform pin with the checkout used by Model tests.

These tests use temporary local Git repositories, not ROS or motion hardware.
"""
from pathlib import Path
import subprocess

import evidence_common as ev


NAME = "openamr_platform_sw"


def checkout(workspace, relative="src/openamr-platform-sw"):
    repo = workspace / relative
    repo.mkdir(parents=True)
    subprocess.run(["git", "init", "-q", str(repo)], check=True)
    subprocess.run(
        ["git", "-C", str(repo), "-c", "user.name=Test Fixture",
         "-c", "user.email=fixture@example.invalid", "-c", "commit.gpgsign=false",
         "commit", "-q", "--allow-empty", "-m", "fixture"], check=True)
    return repo, ev.git_head(str(repo))


def pins(workspace, head):
    path = workspace / "pins.yaml"
    path.write_text(f'schema_version: 1\npins:\n  {NAME}: "{head}"\n')
    return str(path)


def test_platform_uses_the_workflow_checkout_path(tmp_path):
    assert Path(ev.upstream_checkout(str(tmp_path), NAME)) == tmp_path / "src/openamr-platform-sw"


def test_matching_platform_is_compared_not_skipped(tmp_path):
    repo, head = checkout(tmp_path)
    result, details = ev.pins_vs_checkouts(str(tmp_path), pins(tmp_path, head), required=())
    assert result == "PASS"
    assert details["compared"] == [NAME]
    assert details["not_compared"] == []
    assert details["pins"][NAME]["checkout"] == str(repo)
    assert details["pins"][NAME]["checkout_head"] == head


def test_wrong_platform_pin_fails_and_keeps_both_hashes(tmp_path):
    _, head = checkout(tmp_path)
    wrong = "f" * 40
    assert head != wrong
    result, details = ev.pins_vs_checkouts(str(tmp_path), pins(tmp_path, wrong), required=())
    assert result == "FAIL"
    assert head in details["reason"] and wrong in details["reason"]
    assert details["pins"][NAME]["result"] == "FAIL"


def test_dirty_platform_checkout_fails(tmp_path):
    repo, head = checkout(tmp_path)
    (repo / "local_edit.txt").write_text("uncommitted input\n")
    result, details = ev.pin_vs_checkout(str(tmp_path), NAME, pins(tmp_path, head))
    assert result == "FAIL" and "local changes" in details["reason"]


def test_absent_platform_remains_explicitly_not_compared(tmp_path):
    result, details = ev.pins_vs_checkouts(str(tmp_path), pins(tmp_path, "a" * 40), required=())
    assert result == "SKIP"
    assert details["not_compared"] == [NAME]
    assert "src/openamr-platform-sw" in details["pins"][NAME]["reason"]


def test_decoy_at_old_path_cannot_hide_real_platform_drift(tmp_path):
    repo, head = checkout(tmp_path)
    decoy, decoy_head = checkout(tmp_path, "src/upstream/openamr_platform_sw")
    subprocess.run(
        ["git", "-C", str(decoy), "-c", "user.name=Test Fixture",
         "-c", "user.email=fixture@example.invalid", "-c", "commit.gpgsign=false",
         "commit", "-q", "--allow-empty", "-m", "different fixture"], check=True)
    decoy_head = ev.git_head(str(decoy))
    assert head != decoy_head
    result, details = ev.pin_vs_checkout(str(tmp_path), NAME, pins(tmp_path, decoy_head))
    assert result == "FAIL"
    assert details["checkout_head"] == head
    assert str(repo) == ev.upstream_checkout(str(tmp_path), NAME)


def test_openarm_paths_are_unchanged(tmp_path):
    for name in ("openarm_description", "openarm_ros2"):
        assert Path(ev.upstream_checkout(str(tmp_path), name)) == tmp_path / "src/upstream" / name


def test_platform_provenance_reads_the_actual_checkout(tmp_path):
    _, head = checkout(tmp_path)
    commit, source = ev.upstream_commit_and_source(str(tmp_path), NAME, pins(tmp_path, head))
    assert commit == head
    assert source == ev.GIT_SOURCE
