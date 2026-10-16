"""T09: every massed link has a physically possible inertial.

For each link with an <inertial>:
  - mass is finite and > 0;
  - Ixx, Iyy, Izz > 0;
  - the principal moments (eigenvalues of the full 3x3 tensor, so the products of inertia count) are all > 0, i.e. the
    tensor is positive-definite;
  - each principal moment <= the sum of the other two (the triangle inequality of a rigid body).

The triangle inequality is tested with a relative tolerance, TRIANGLE_RTOL of the largest moment: a thin plate has
Izz = Ixx + Iyy exactly, and a CAD export rounds. Nothing else is toleranced: a moment that is zero or negative fails.
Rotating the inertial frame (<origin rpy>) does not change the eigenvalues, so the origin is not needed.
"""
import math

import numpy as np
import pytest

from conftest import MATRIX, expand, write_sidecar

TRIANGLE_RTOL = 1e-6


def _tensor(inertial):
    inertia = inertial.find("inertia")
    g = lambda k: float(inertia.get(k, 0.0))  # noqa: E731
    return np.array([[g("ixx"), g("ixy"), g("ixz")],
                     [g("ixy"), g("iyy"), g("iyz")],
                     [g("ixz"), g("iyz"), g("izz")]])


def inertia_problems(link_name, inertial):
    """FAIL strings for one <inertial> (empty list = fine)."""
    problems = []
    mass = float(inertial.find("mass").get("value"))
    if not math.isfinite(mass) or mass <= 0:
        problems.append(f"{link_name}: mass {mass} is not > 0")
    tensor = _tensor(inertial)
    if not np.isfinite(tensor).all():
        return problems + [f"{link_name}: inertia tensor has a non-finite entry"]
    if not (np.diag(tensor) > 0).all():
        problems.append(f"{link_name}: Ixx, Iyy, Izz = {np.diag(tensor).tolist()} are not all > 0")
    moments = np.linalg.eigvalsh(tensor)                  # ascending
    if not (moments > 0).all():
        problems.append(f"{link_name}: principal moments {moments.tolist()} are not all > 0 (not positive-definite)")
    slack = TRIANGLE_RTOL * moments.max()
    a, b, c = moments
    for lhs, rest, tag in ((a, b + c, "smallest"), (b, a + c, "middle"), (c, a + b, "largest")):
        if lhs > rest + slack:
            problems.append(f"{link_name}: {tag} principal moment {lhs:.6g} exceeds the sum of the other two {rest:.6g}")
    return problems


def massed_links(root):
    return [(link.get("name"), link.find("inertial")) for link in root.findall("link") if link.find("inertial") is not None]


def test_t09_every_massed_link_in_every_expansion_is_physically_possible():
    problems, checked, names = [], 0, set()
    for base_plate, arm_package in MATRIX:
        for name, inertial in massed_links(expand(base_plate=base_plate, arm_package=arm_package)):
            checked += 1
            names.add(name)
            problems += [f"[{base_plate}/{arm_package}] {p}" for p in inertia_problems(name, inertial)]
    write_sidecar("T09", "inertia", "massed links: mass > 0, principal moments > 0, triangle inequality, tensor PD",
                  "FAIL" if problems else "PASS",
                  {"massed_links": len(names), "link_instances_checked": checked, "failures": problems,
                   "triangle_rtol": TRIANGLE_RTOL})
    assert not problems, "\n" + "\n".join(problems)


def test_t09_the_default_expansion_has_massed_links_from_all_three_owners():
    # guards against the check passing because it found nothing to check
    names = [n for n, _ in massed_links(expand())]
    assert "base_link" in names and "lift_column_base_link" in names and "openarm_left_link1" in names
    assert len(names) >= 30


# --- the checker on synthetic inertials: it must be able to fail --------------------------------------------------------

def _inertial(mass=1.0, ixx=1.0, iyy=1.0, izz=1.0, ixy=0.0, ixz=0.0, iyz=0.0):
    import xml.etree.ElementTree as ET
    return ET.fromstring(f"<inertial><mass value='{mass}'/><inertia ixx='{ixx}' iyy='{iyy}' izz='{izz}' "
                         f"ixy='{ixy}' ixz='{ixz}' iyz='{iyz}'/></inertial>")


def test_t09_checker_accepts_a_cube_a_thin_plate_and_a_rotated_tensor():
    assert not inertia_problems("cube", _inertial())
    assert not inertia_problems("plate", _inertial(ixx=1.0, iyy=1.0, izz=2.0))        # Izz = Ixx + Iyy exactly
    assert not inertia_problems("rotated", _inertial(ixx=1.5, iyy=1.5, izz=1.0, ixy=0.5))   # principal moments 1, 1, 2


@pytest.mark.parametrize("kwargs,needle", [
    ({"mass": 0.0}, "mass 0.0 is not > 0"),
    ({"mass": -1.0}, "is not > 0"),
    ({"mass": float("nan")}, "is not > 0"),
    ({"ixx": 0.0}, "not all > 0"),
    ({"iyy": -1.0, "ixx": 3.0, "izz": 3.0}, "not all > 0"),
    ({"ixx": 1.0, "iyy": 1.0, "izz": 3.0}, "exceeds the sum of the other two"),         # triangle inequality
    ({"ixx": 1.0, "iyy": 1.0, "izz": 1.0, "ixy": 1.5}, "not positive-definite"),        # diagonal fine, tensor is not
    ({"ixx": float("inf")}, "non-finite"),
])
def test_t09_checker_fails(kwargs, needle):
    problems = inertia_problems("x", _inertial(**kwargs))
    assert any(needle in p for p in problems), problems
