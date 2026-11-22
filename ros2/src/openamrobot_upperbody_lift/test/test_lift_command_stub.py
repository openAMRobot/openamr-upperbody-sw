# SPDX-License-Identifier: MIT
"""The lift command path is a stub pending the I8 lift contract: every call refuses, and the package opens no CAN."""
import ast
import os

import pytest

import openamrobot_upperbody_lift as package
from openamrobot_upperbody_lift.command_stub import LiftCommandInterface, LiftCommandUnavailable


@pytest.mark.parametrize("call", [lambda i: i.stop(), lambda i: i.move_to_stop("L1175"), lambda i: i.clear_fault(), lambda i: i.start_homing()])
def test_every_command_refuses(call):
    with pytest.raises(LiftCommandUnavailable, match="I8 lift contract"):
        call(LiftCommandInterface())


def test_the_package_never_opens_can():
    """I8 rev 3, decision D17: the Jetson does not talk to the lift node on CAN3. Read the imports, not the prose."""
    root = os.path.dirname(package.__file__)
    for name in sorted(os.listdir(root)):
        if not name.endswith(".py"):
            continue
        with open(os.path.join(root, name)) as f:
            tree = ast.parse(f.read())
        for node in ast.walk(tree):
            modules = [a.name for a in node.names] if isinstance(node, ast.Import) else [node.module or ""] if isinstance(node, ast.ImportFrom) else []
            for module in modules:
                assert module.split(".")[0] not in ("can", "socket", "canopen", "pyvit"), f"{name} imports {module}"
