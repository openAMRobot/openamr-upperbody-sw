# SPDX-License-Identifier: MIT
"""T04: every expansion honours config/frame_contract.yaml.

  R   present, with the contract's parent and type            -> FAIL if not
  C   present iff its condition holds                         -> FAIL otherwise
  W   expected from another owner; missing is a WARN          -> collected into a JSON sidecar
  any link in our namespaces (lift_*, arm_profile_*, head_camera_*, *_wrist_camera_*, so101_*) that the contract does not list -> FAIL
  collision_free: true  -> the link has no <collision>
  consumed_from: frames of another owner that this file does not declare and does not check (the near-field sensors, platform-sw's):
                 reported in the sidecar as observed or not, never a WARN and never a FAIL (P15: "take them from there")

Entries with `variants` (base_footprint) give type and parent per arg value; exactly one variant must match each expansion.
The checker is a pure function of (URDF root, args, contract) so that it is itself tested on synthetic URDFs below.

Sidecar: T04_frame_contract.json in the evidence directory ($OPENAMR_EVIDENCE_DIR, default <workspace>/evidence;
never a temp dir, see evidence_common.evidence_dir).
"""
import fnmatch
import os
import re
import warnings
import xml.etree.ElementTree as ET

import pytest

from conftest import ARG_DEFAULTS, ARM_PACKAGES, BASELINE, MATRIX, SHARE, expand, load_yaml, write_sidecar

OUR_NAMESPACES = ["lift_*", "arm_profile_*", "head_camera_*", "*_wrist_camera_*", "so101_*"]
_COND = re.compile(r"^\s*(\w+)\s*==\s*(\S+)\s*$")

CONTRACT = load_yaml(os.path.join(SHARE, "config", "frame_contract.yaml"))
# Every base plate x arm package, plus the planar flag for each arm_package (the flag changes base_footprint's parent)
EXPANSIONS = [(p, a, "false") for p, a in MATRIX] + [(BASELINE["base_plate"], a, "true") for a in ARM_PACKAGES]


def label(base_plate, arm_package, planar):
    return f"{base_plate}-{arm_package}-planar_{planar}"


def holds(condition, args):
    """Evaluate `<arg> == <value>` over the args (strings). None is always true."""
    if condition is None:
        return True
    m = _COND.match(condition)
    if not m:
        raise ValueError(f"unparseable contract condition: {condition!r}")
    arg, value = m.groups()
    if arg not in args:
        raise ValueError(f"contract condition names unknown arg {arg!r}: {condition!r}")
    return args[arg] == value


class Facts:
    """Links, joints, and the parent/type of every link, read from an expanded URDF."""

    def __init__(self, root):
        self.links = {e.get("name"): e for e in root.findall("link")}
        self.joints = {e.get("name"): e for e in root.findall("joint")}
        self.attached_by = {j.find("child").get("link"): j for j in self.joints.values()}

    def describe(self, kind, name):
        """(type, parent) of a link or joint, or None if absent. Link type = type of its parent joint, else 'root'."""
        if kind == "joint":
            j = self.joints.get(name)
            return None if j is None else (j.get("type"), j.find("parent").get("link"))
        if name not in self.links:
            return None
        j = self.attached_by.get(name)
        return ("root", None) if j is None else (j.get("type"), j.find("parent").get("link"))


def expected_type_parent(entry, args):
    """(type, parent) the contract expects for this entry under these args."""
    if "variants" not in entry:
        return entry["type"], entry["parent"]
    hits = [v for v in entry["variants"] if holds(v["when"], args)]
    if len(hits) != 1:
        raise ValueError(f"{entry['name']}: {len(hits)} variants match {args} (need exactly 1)")
    return hits[0]["type"], hits[0]["parent"]


def check_contract(root, args, contract):
    """-> (fails, warns): lists of readable strings. `args` must hold every arg a condition mentions."""
    facts = Facts(root)
    fails, warns = [], []
    for entry in contract["entries"]:
        kind, name, req = entry["kind"], entry["name"], entry["req"]
        found = facts.describe(kind, name)
        applies = req == "R" or req == "W" or holds(entry["condition"], args)
        if req == "C" and not applies:
            if found is not None:
                fails.append(f"{kind} {name}: present but its condition is false ({entry['condition']})")
            continue
        if found is None:
            (warns if req == "W" else fails).append(f"{kind} {name}: missing ({req})")
            continue
        try:
            want_type, want_parent = expected_type_parent(entry, args)
        except ValueError as err:
            fails.append(str(err))
            continue
        if found != (want_type, want_parent):
            fails.append(f"{kind} {name}: type/parent {found}, contract says {(want_type, want_parent)}")
        if entry.get("collision_free") and facts.links[name].find("collision") is not None:
            fails.append(f"link {name}: contract says collision_free but it has a <collision>")
    listed = {e["name"] for e in contract["entries"]}
    for name in sorted(facts.links):
        if name not in listed and any(fnmatch.fnmatchcase(name, pat) for pat in OUR_NAMESPACES):
            fails.append(f"link {name}: in our namespace but not in the contract")
    return fails, warns


def full_args(base_plate, arm_package, planar):
    return {**ARG_DEFAULTS, "base_plate": base_plate, "arm_package": arm_package, "enable_planar_base": planar}


@pytest.mark.parametrize("base_plate,arm_package,planar", EXPANSIONS, ids=[label(*e) for e in EXPANSIONS])
def test_t04_expansion_honours_the_contract(base_plate, arm_package, planar):
    root = expand(base_plate=base_plate, arm_package=arm_package, enable_planar_base=planar)
    fails, _ = check_contract(root, full_args(base_plate, arm_package, planar), CONTRACT)
    assert not fails, "\n" + "\n".join(fails)


def test_t04_there_is_no_w_entry_and_the_near_field_frames_are_consumed_not_declared():
    """P15: the six near-field frames this contract listed as `W` (declared-but-absent: a WARN in every expansion since S4) are
    DELETED, not renamed. The layout is platform-sw's (4 x us_*, 2 x tof_*) and arrives with the included base file; the contract
    says so in `consumed_from` and the sidecar says which of them each expansion contains. T04's result is PASS because there is
    nothing of another owner's left for it to wait for, and absent frames would be reported, not warned: they are not ours."""
    assert not [e["name"] for e in CONTRACT["entries"] if e["req"] == "W"], "a W entry is back: another owner's frame was declared again"
    consumed = CONTRACT["consumed_from"]
    assert [c["owner"] for c in consumed] == ["platform-sw"]
    frames = consumed[0]["frames"]
    assert frames == ["us_front_left", "us_front_right", "us_rear_left", "us_rear_right", "tof_front", "tof_rear", "imu_link", "base_camera_link"]
    declared = {e["name"] for e in CONTRACT["entries"]}
    assert not declared & set(frames), "a consumed frame is also declared by this contract"
    stale = {"tof_front_left_link", "tof_front_right_link", "tof_rear_left_link", "tof_rear_right_link",
             "ultrasonic_front_link", "ultrasonic_rear_link"}
    assert not declared & stale, "one of the six retired near-field names is still in the contract"

    fails_all, observed = [], {}
    for e in EXPANSIONS:
        root = expand(base_plate=e[0], arm_package=e[1], enable_planar_base=e[2])
        fails, warns = check_contract(root, full_args(*e), CONTRACT)
        fails_all += [f"{label(*e)}: {f}" for f in fails]
        assert not warns, warns
        present = {n.get("name") for n in root.findall("link")}
        observed[label(*e)] = [f for f in frames if f in present]
    assert not fails_all, "\n".join(fails_all)
    write_sidecar("T04", "frame_contract", "frame contract", "PASS",
                  {"expansions": len(EXPANSIONS), "missing_W": [], "entries": len(CONTRACT["entries"]),
                   "consumed_from": {"owner": consumed[0]["owner"], "contract": consumed[0]["contract"], "frames": frames,
                                     "state": "merged upstream (platform-sw main 935efc7) and present in the included base file",
                                     "observed_in_every_expansion": all(v == frames for v in observed.values()),
                                     "observed": observed[label(*EXPANSIONS[0])]},
                   "owner_of_the_contract": "platform-sw (near-field frames); this repository declares none of them"})


# --- the contract file itself ---------------------------------------------------------------------------------------

def test_t04_contract_is_well_formed():
    seen = set()
    for e in CONTRACT["entries"]:
        key = (e["kind"], e["name"])
        assert key not in seen, f"duplicate contract entry {key}"
        seen.add(key)
        assert e["kind"] in ("link", "joint") and e["req"] in ("R", "C", "W"), e
        assert (e["req"] == "C") == (e["condition"] is not None), f"{e['name']}: condition iff req C"
        holds(e["condition"], ARG_DEFAULTS)                      # parses and names a real arg
        if "variants" in e:
            assert "type" not in e and "parent" not in e, f"{e['name']}: variants replace type/parent"
            for v in e["variants"]:
                assert {"when", "type", "parent"} <= set(v), e
                holds(v["when"], ARG_DEFAULTS)
        else:
            assert "type" in e and "parent" in e, e
        if e.get("collision_free"):
            assert e.get("collision_free_reason"), f"{e['name']}: collision_free needs a reason"
            assert e["kind"] == "link"


def test_t04_every_link_attached_by_contract_has_its_parent_in_the_contract_or_is_upstream():
    """A contract parent must itself be a frame the contract knows (or the planar `world`)."""
    names = {e["name"] for e in CONTRACT["entries"] if e["kind"] == "link"}
    for e in CONTRACT["entries"]:
        parents = [v["parent"] for v in e["variants"]] if "variants" in e else [e["parent"]]
        for p in parents:
            assert p is None or p in names, f"{e['name']}: parent {p!r} is not a contract link"


def test_t04_base_footprint_is_conditionally_a_root_and_base_link_hangs_from_it():
    entry = next(e for e in CONTRACT["entries"] if e["name"] == "base_footprint")
    assert expected_type_parent(entry, {"enable_planar_base": "false"}) == ("root", None)
    assert expected_type_parent(entry, {"enable_planar_base": "true"}) == ("planar", "world")
    link = next(e for e in CONTRACT["entries"] if e["name"] == "base_link")
    assert (link["type"], link["parent"]) == ("fixed", "base_footprint")


# --- the checker on synthetic URDFs: it must be able to fail ----------------------------------------------------------

def _urdf(links, joints, collision_on=()):
    parts = []
    for n in links:
        c = "<collision><geometry><box size='1 1 1'/></geometry></collision>" if n in collision_on else ""
        parts.append(f"<link name='{n}'>{c}</link>")
    for n, t, p, c in joints:
        parts.append(f"<joint name='{n}' type='{t}'><parent link='{p}'/><child link='{c}'/></joint>")
    return ET.fromstring("<robot>" + "".join(parts) + "</robot>")


MINI = {"entries": [
    {"name": "base_link", "kind": "link", "req": "R", "condition": None,
     "variants": [{"when": "p == false", "type": "root", "parent": None},
                  {"when": "p == true", "type": "planar", "parent": "world"}]},
    {"name": "lift_a_link", "kind": "link", "type": "fixed", "parent": "base_link", "req": "R", "condition": None,
     "collision_free": True, "collision_free_reason": "x"},
    {"name": "cam_link", "kind": "link", "type": "fixed", "parent": "lift_a_link", "req": "C", "condition": "k == on"},
    {"name": "tof_link", "kind": "link", "type": "fixed", "parent": "base_link", "req": "W", "condition": None},
    {"name": "world", "kind": "link", "type": "root", "parent": None, "req": "C", "condition": "p == true"},
    {"name": "lift_joint_x", "kind": "joint", "type": "fixed", "parent": "base_link", "req": "R", "condition": None},
]}
GOOD = dict(links=["base_link", "lift_a_link"], joints=[("lift_joint_x", "fixed", "base_link", "lift_a_link")])


def _check(links, joints, args, collision_on=(), contract=MINI):
    return check_contract(_urdf(links, joints, collision_on), args, contract)


def test_checker_passes_a_conforming_urdf_and_warns_on_a_missing_w_frame():
    fails, warns = _check(**GOOD, args={"p": "false", "k": "off"})
    assert not fails and warns == ["link tof_link: missing (W)"]


def test_checker_fails_a_missing_r_frame():
    fails, _ = _check(["base_link"], [], {"p": "false", "k": "off"})
    assert any("lift_a_link: missing (R)" in f for f in fails)


def test_checker_fails_a_wrong_parent_and_a_wrong_type():
    fails, _ = _check(["base_link", "lift_a_link", "x"], [("lift_joint_x", "fixed", "x", "lift_a_link")],
                      {"p": "false", "k": "off"})
    assert any("lift_joint_x" in f and "contract says" in f for f in fails)
    fails, _ = _check(GOOD["links"], [("lift_joint_x", "revolute", "base_link", "lift_a_link")], {"p": "false", "k": "off"})
    assert any("lift_joint_x" in f and "revolute" in f for f in fails)


def test_checker_fails_a_conditional_frame_when_present_without_its_condition_and_when_absent_with_it():
    extra = dict(links=GOOD["links"] + ["cam_link"], joints=GOOD["joints"] + [("cj", "fixed", "lift_a_link", "cam_link")])
    fails, _ = _check(**extra, args={"p": "false", "k": "off"})
    assert any("cam_link: present but its condition is false" in f for f in fails)
    fails, _ = _check(**GOOD, args={"p": "false", "k": "on"})
    assert any("cam_link: missing (C)" in f for f in fails)


def test_checker_fails_an_unlisted_frame_in_our_namespaces_only():
    for name, bad in (("lift_extra_link", True), ("head_camera_link", True), ("left_wrist_camera_extra", True),
                      ("so101_thing", True), ("some_other_link", False)):
        fails, _ = _check(GOOD["links"] + [name], GOOD["joints"], {"p": "false", "k": "off"})
        assert any(f"{name}: in our namespace" in f for f in fails) == bad, name


def test_checker_applies_the_right_base_link_variant_and_fails_the_wrong_one():
    planar = dict(links=["world", "base_link", "lift_a_link"],
                  joints=[("bp", "planar", "world", "base_link"), ("lift_joint_x", "fixed", "base_link", "lift_a_link")])
    assert not _check(**planar, args={"p": "true", "k": "off"})[0]
    # the same tree is wrong when the arg says planar is off: base_link is not a root, world must be absent
    fails, _ = _check(**planar, args={"p": "false", "k": "off"})
    assert any("base_link" in f for f in fails) and any("world: present but" in f for f in fails)
    # and a default (rootless-planar) tree is wrong when the arg says planar is on
    fails, _ = _check(**GOOD, args={"p": "true", "k": "off"})
    assert any("base_link" in f for f in fails) and any("world: missing" in f for f in fails)


def test_checker_fails_when_no_or_several_variants_match():
    with pytest.raises(ValueError):
        expected_type_parent(MINI["entries"][0], {"p": "maybe"})
    both = {"name": "b", "variants": [{"when": "p == true", "type": "a", "parent": None},
                                      {"when": "p == true", "type": "b", "parent": None}]}
    with pytest.raises(ValueError):
        expected_type_parent(both, {"p": "true"})
    fails, _ = _check(**GOOD, args={"p": "maybe", "k": "off"})
    assert any("variants match" in f for f in fails)


def test_checker_fails_a_collision_on_a_collision_free_link():
    fails, _ = _check(**GOOD, args={"p": "false", "k": "off"}, collision_on=("lift_a_link",))
    assert any("collision_free" in f for f in fails)
