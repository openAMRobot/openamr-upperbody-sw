"""scripts/datums_report.py prints the datums by status and source plus the placeholder list, and exits 0."""
import os
import subprocess
import sys

from conftest import SHARE, load_datums

SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, "scripts", "datums_report.py")


def _run(*args):
    return subprocess.run([sys.executable, SCRIPT, *args], capture_output=True, text=True)


def test_report_exits_zero_and_lists_every_placeholder():
    result = _run("--datums", os.path.join(SHARE, "config", "datums.yaml"))
    assert result.returncode == 0, result.stderr
    datums = load_datums()["datums"]
    placeholders = [n for n, d in datums.items() if d["status"] == "placeholder"]
    section = result.stdout.split("Placeholders (")[1]
    assert section.startswith(f"{len(placeholders)} of {len(datums)})")
    for name in placeholders:
        assert name in section
    for status in ("placeholder", "provisional", "verified"):
        assert f"  {status}" in result.stdout
    pending = load_datums().get("pending_reference", {})
    assert f"pending_reference ({len(pending)})" in result.stdout
    for name, step in pending.items():
        assert f"{name}" in result.stdout and step in result.stdout


def test_report_with_no_flag_finds_the_package_datums():
    result = _run()
    assert result.returncode == 0, result.stderr
    assert "datums.yaml revision" in result.stdout
