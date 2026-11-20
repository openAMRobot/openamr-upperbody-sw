# SPDX-License-Identifier: MIT
"""The installed package against its source tree (openamr-upperbody-sw#88: tests that import from the source tree do
not prove the package installs).

    python3 check_installed.py INSTALL_PREFIX PACKAGE_SOURCE_DIR

INSTALL_PREFIX is the package's install prefix (colcon: install/openamrobot_control_room). Fails (exit 1) when a
module of openamrobot_control_room/, a script of scripts/ (installed and executable) or a file of launch/, config/
or rviz/ is missing from the install, or when nothing at all was compared. Prints what it compared.
"""
import glob
import os
import sys

PACKAGE = "openamrobot_control_room"


def problems(prefix, source):
    found, missing = [], []
    site = glob.glob(os.path.join(prefix, "lib", "python3*", "site-packages", PACKAGE)) + \
        glob.glob(os.path.join(prefix, "local", "lib", "python3*", "dist-packages", PACKAGE))
    for module in sorted(glob.glob(os.path.join(source, PACKAGE, "*.py"))):
        name = os.path.basename(module)
        hits = [os.path.join(d, name) for d in site if os.path.exists(os.path.join(d, name))]
        (found if hits else missing).append(f"module {name}")
    for script in sorted(glob.glob(os.path.join(source, "scripts", "*.py"))):
        target = os.path.join(prefix, "lib", PACKAGE, os.path.basename(script))
        ok = os.path.exists(target) and os.access(target, os.X_OK)
        (found if ok else missing).append(f"script {os.path.basename(script)}{'' if ok else ' (missing or not executable)'}")
    for folder in ("launch", "config", "rviz"):
        for path in sorted(glob.glob(os.path.join(source, folder, "*"))):
            target = os.path.join(prefix, "share", PACKAGE, folder, os.path.basename(path))
            (found if os.path.exists(target) else missing).append(f"{folder}/{os.path.basename(path)}")
    if not found and not missing:
        missing.append(f"nothing to compare in {source}")
    if not found:
        missing.append(f"nothing of the package found under {prefix}")
    return found, missing


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    found, missing = problems(argv[0], argv[1])
    for item in found:
        print(f"installed: {item}")
    for item in missing:
        print(f"MISSING: {item}")
    print(f"{len(found)} installed, {len(missing)} missing")
    return 1 if missing else 0


if __name__ == "__main__":
    sys.exit(main())
