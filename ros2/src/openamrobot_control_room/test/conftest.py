# SPDX-License-Identifier: MIT
"""Paths for the control room tests. The package's Python module is used from the source tree."""
import os
import sys

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPOSITORY = os.path.abspath(os.path.join(PACKAGE_DIR, "..", "..", ".."))
sys.path.insert(0, PACKAGE_DIR)
TRACKING = "openamr-upperbody-sw#83"


def package_path(*parts):
    return os.path.join(PACKAGE_DIR, *parts)


def read(*parts):
    with open(package_path(*parts), encoding="utf-8") as f:
        return f.read()
