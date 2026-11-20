# SPDX-License-Identifier: MIT
"""The upstream pin watch reports a pin that PyPI stopped serving, without network access in the test."""
import importlib.util
import pathlib

import pytest

TOOL = pathlib.Path(__file__).resolve().parents[1] / "tools" / "check_upstream_pins.py"
spec = importlib.util.spec_from_file_location("check_upstream_pins", TOOL)
cup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cup)

SHA = "c3b330ebd984a3bce6d6482e0047eae5c5333fedd49b30b9b6df863a086b35f7"


def record(sha=SHA, yanked=False):
    return {"urls": [{"filename": "xacro-2.1.1-py3-none-any.whl", "digests": {"sha256": sha}, "yanked": yanked}]}


@pytest.fixture
def files(tmp_path):
    hashed = tmp_path / "requirements-xacro.txt"
    hashed.write_text(f"# comment\nxacro==2.1.1 --hash=sha256:{SHA}\n")
    plain = tmp_path / "requirements.txt"
    plain.write_text("numpy==2.5.3        # BSD-3-Clause\n")
    return (hashed, plain)


def run(files, answers, tmp_path):
    summary = tmp_path / "summary.md"
    rc = cup.main(["--summary", str(summary)], fetch=lambda name, version: answers[name], files=files)
    return rc, summary.read_text()


def test_every_pin_served(files, tmp_path):
    rc, text = run(files, {"xacro": record(), "numpy": record(sha="0" * 64)}, tmp_path)
    assert rc == 0 and "all pins served" in text


@pytest.mark.parametrize("answer, reason", [(None, "404"), (record(sha="1" * 64), "no file with the pinned sha256"),
                                            (record(yanked=True), "yanked")])
def test_hashed_pin_not_served(files, tmp_path, answer, reason):
    rc, text = run(files, {"xacro": answer, "numpy": record(sha="0" * 64)}, tmp_path)
    assert rc == 1 and "NOT SERVED" in text and reason in text


def test_version_pin_all_yanked(files, tmp_path):
    rc, text = run(files, {"xacro": record(), "numpy": record(sha="0" * 64, yanked=True)}, tmp_path)
    assert rc == 1 and "every file is yanked" in text


def test_network_error_is_blocked_not_a_pass(files, tmp_path):
    def fetch(name, version):
        raise cup.Unreachable("offline")
    summary = tmp_path / "s.md"
    assert cup.main(["--summary", str(summary)], fetch=fetch, files=files) == 2
    assert "BLOCKED" in summary.read_text()


def test_repository_pin_files_parse():
    """Both pin files of this repository hold only exact pins; the xacro file holds one hashed pin."""
    hashed, plain = (cup.parse(p.read_text()) for p in cup.FILES)
    assert len(hashed) == 1 and hashed[0]["name"] == "xacro" and hashed[0]["sha256"]
    assert plain and all(p["sha256"] is None for p in plain)
