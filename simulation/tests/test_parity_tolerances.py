# SPDX-License-Identifier: MIT
"""Every parity tolerance is at most one tenth of the precision its source states, and names that precision."""
import pytest

from openamr_sim.parity import tolerances

TOL = tolerances()


@pytest.mark.parametrize("name", sorted(TOL))
def test_tolerance_is_finer_than_source(name):
    t = TOL[name]
    precision = t["source_precision"]
    assert precision.get("note"), f"{name}: no statement of source precision"
    if precision.get("relative"):
        assert "rel" in t and t["rel"] <= precision["value"] / 10
    else:
        assert "abs" in t and t["abs"] <= precision["value"] / 10
        if "rel" in t:
            assert t["rel"] <= 1e-3
