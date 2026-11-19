# SPDX-License-Identifier: MIT
"""tools/check_installed.py on made-up install trees: a complete one passes, one without a script, one with a script
that is not executable and an empty one fail (a check that finds nothing must fail, openamr-upperbody-sw#88)."""
import importlib.util
import os
import shutil

from conftest import PACKAGE_DIR, package_path

spec = importlib.util.spec_from_file_location("check_installed", package_path("tools", "check_installed.py"))
check = importlib.util.module_from_spec(spec)
spec.loader.exec_module(check)


def _install(tmp_path, skip=(), mode=0o755):
    prefix = tmp_path / "install"
    site = prefix / "lib" / "python3.12" / "site-packages" / check.PACKAGE
    site.mkdir(parents=True)
    for name in os.listdir(os.path.join(PACKAGE_DIR, check.PACKAGE)):
        if name.endswith(".py"):
            shutil.copy(os.path.join(PACKAGE_DIR, check.PACKAGE, name), site / name)
    lib = prefix / "lib" / check.PACKAGE
    lib.mkdir(parents=True)
    for name in os.listdir(package_path("scripts")):
        if name not in skip:
            shutil.copy(package_path("scripts", name), lib / name)
            os.chmod(lib / name, mode)
    for folder in ("launch", "config", "rviz"):
        shutil.copytree(package_path(folder), prefix / "share" / check.PACKAGE / folder)
    return str(prefix)


def test_complete_install_passes(tmp_path):
    assert check.main([_install(tmp_path), PACKAGE_DIR]) == 0


def test_missing_script_fails(tmp_path):
    found, missing = check.problems(_install(tmp_path, skip=("control_room_replay.py",)), PACKAGE_DIR)
    assert missing == ["script control_room_replay.py (missing or not executable)"]


def test_script_not_executable_fails(tmp_path):
    assert check.main([_install(tmp_path, mode=0o644), PACKAGE_DIR]) == 1


def test_empty_install_fails(tmp_path):
    (tmp_path / "empty").mkdir()
    assert check.main([str(tmp_path / "empty"), PACKAGE_DIR]) == 1
