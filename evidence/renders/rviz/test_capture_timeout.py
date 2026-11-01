# SPDX-License-Identifier: MIT
"""The optional screenshot helper must terminate a stalled X-grab subprocess."""
import importlib.util
from pathlib import Path
import subprocess
import time

import pytest


def test_real_hung_capture_is_bounded(tmp_path):
    path = Path(__file__).with_name("capture_rviz.py")
    spec = importlib.util.spec_from_file_location("capture", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    (tmp_path / "xgrab.py").write_text("import time\ntime.sleep(2)\n")
    module.HERE = str(tmp_path)
    start = time.monotonic()
    with pytest.raises(subprocess.TimeoutExpired):
        module.capture_window("0x1", str(tmp_path / "out.png"), timeout=0.15)
    assert time.monotonic() - start < 1
