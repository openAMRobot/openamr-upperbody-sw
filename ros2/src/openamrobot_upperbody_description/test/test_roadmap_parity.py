# SPDX-License-Identifier: MIT
"""Roadmap parity (convergence phase 3 of openamrobot-docs scripts/roadmap_model): the upper body of the combined model is the roadmap model,
within 0.5 mm and 0.2 deg, in every configuration of the parity report (bp000 at L1000 / L1175 / L1350, bp050 at L1000; hp30; arms at zero).

The roadmap side is a pinned fixture (test/fixtures/roadmap_parity.json), written from an openamrobot-docs checkout by
scripts/roadmap_parity_dump.js: the part groups of the upper body as the roadmap pages build them, mesh by mesh, and the arm rig's frames.
Which URDF link is which roadmap part, what the roadmap draws for looks only, and which parts have no URDF counterpart is
test/fixtures/roadmap_parity_map.yaml, each exclusion with its reason. For every mapped item the world box of the URDF visuals (centre = pose,
extents = primitive size) is compared with the world box of the roadmap meshes; for the OpenArm links the link frames. A `pending` entry is
an intended, undecided difference: it is reported and must still differ, so a closed one cannot linger in the allow-list.

    python3 test/test_roadmap_parity.py      prints the comparison table (needs the installed package, as the tests do)
"""
import json
import math
import os
import warnings

import numpy as np
import pytest
import yaml

from conftest import FIXTURES, STOPS, expand
from urdf_utils import pose_to_matrix, zero_pose_poses

FIXTURE = os.path.join(FIXTURES, "roadmap_parity.json")
MAP = os.path.join(FIXTURES, "roadmap_parity_map.yaml")
with open(FIXTURE) as _f:
    DUMP = json.load(_f)
with open(MAP) as _f:
    PARITY = yaml.safe_load(_f)
TOL_M, TOL_DEG = PARITY["tolerance"]["mm"] / 1000.0, PARITY["tolerance"]["deg"]
CONFIGS = DUMP["configurations"]          # {"bp000 L1000": {base_plate, lift_joint}, ...}
MAPPED = PARITY["mapped"]
ITEM_IDS = [" + ".join(e["urdf"]) for e in MAPPED]


def _urdf(base_plate):
    return expand(base_plate=base_plate, arm_package="openarm")


def _vector(text, default="0 0 0"):
    return [float(v) for v in (text or default).split()]


def _origin(element):
    o = element.find("origin")
    return pose_to_matrix(_vector(o.get("xyz")), _vector(o.get("rpy"))) if o is not None else np.eye(4)


def _visual_box(visual, world):
    """World axis-aligned box (lo, hi) of one primitive visual; a mesh is not compared (use `frames`)."""
    pose = world @ _origin(visual)
    geometry = visual.find("geometry")[0]
    rot, centre = pose[:3, :3], pose[:3, 3]
    if geometry.tag == "box":
        half = np.abs(rot) @ (np.array(_vector(geometry.get("size"))) / 2)
    elif geometry.tag == "cylinder":
        r, h = float(geometry.get("radius")), float(geometry.get("length"))
        axis = rot[:, 2]
        half = np.abs(axis) * h / 2 + r * np.sqrt(np.clip(1 - axis * axis, 0, None))
    elif geometry.tag == "sphere":
        half = np.full(3, float(geometry.get("radius")))
    else:
        raise AssertionError(f"a {geometry.tag} visual is not compared by its box; map the link under `frames`")
    return centre - half, centre + half


def urdf_box(root, poses, links):
    boxes = [_visual_box(v, poses[link]) for link in links for v in root.findall(f"link[@name='{link}']/visual")]
    assert boxes, f"{links}: no visual"
    return np.min([b[0] for b in boxes], axis=0), np.max([b[1] for b in boxes], axis=0)


def _excluded(mesh, rules):
    return any(("color" in r and mesh["color"] == r["color"]) or ("name" in r and mesh["name"] == r["name"]) for r in rules)


def roadmap_groups(config, ids):
    parts = DUMP["configs"][config]["parts"]
    out = []
    for pid in ids:
        name, _, index = pid.partition("#")
        assert name in parts, f"roadmap part {name} is not in the fixture"
        groups = parts[name]
        out += [groups[int(index)]] if index else groups
    return out


def roadmap_box(config, item):
    rules = PARITY.get("cosmetic", []) + item.get("exclude", [])
    boxes = []
    for group in roadmap_groups(config, item["roadmap"]):
        assert group["meshes"] is not None, f"{item['roadmap']}: recorded as one box, cannot be compared mesh by mesh"
        boxes += [m["box"] for m in group["meshes"] if not _excluded(m, rules)]
    assert boxes, f"{item['roadmap']}: every mesh is excluded"
    return np.min([b[0] for b in boxes], axis=0), np.max([b[1] for b in boxes], axis=0)


def _quat_matrix(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _angle_deg(a, b):
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(a.T @ b) - 1) / 2))))


@pytest.fixture(scope="module")
def poses():
    """{config: (root, {link: world pose})}, the URDF at the fixture's lift joint position."""
    out = {}
    for config, c in CONFIGS.items():
        root = _urdf(c["base_plate"])
        out[config] = (root, zero_pose_poses(root, {"lift_joint": c["lift_joint"]}))
    return out


def compare(item, config, root, world):
    """{dpos_mm, dsize_mm, drot_deg (or None), within} of one mapped item in one configuration."""
    (ulo, uhi), (rlo, rhi) = urdf_box(root, world, item["urdf"]), roadmap_box(config, item)
    dpos = float(np.linalg.norm((rlo + rhi) / 2 - (ulo + uhi) / 2))
    dsize = float(np.max(np.abs((rhi - rlo) - (uhi - ulo))))
    drot = None
    if item.get("rot"):
        link = item["urdf"][0]
        visual = root.find(f"link[@name='{link}']/visual")
        group = roadmap_groups(config, item["roadmap"])[0]
        drot = _angle_deg((world[link] @ _origin(visual))[:3, :3], _quat_matrix(group["q"]))
    within = dpos <= TOL_M and dsize <= TOL_M and (drot is None or drot <= TOL_DEG)
    return {"dpos_mm": dpos * 1000, "dsize_mm": dsize * 1000, "drot_deg": drot, "within": within,
            "size_urdf_mm": (uhi - ulo) * 1000, "size_roadmap_mm": (rhi - rlo) * 1000}


def _describe(item, config, r):
    rot = "" if r["drot_deg"] is None else f", drot {r['drot_deg']:.3f} deg"
    return (f"{' + '.join(item['urdf'])} / {' + '.join(item['roadmap'])} at {config}: dpos {r['dpos_mm']:.2f} mm, dsize {r['dsize_mm']:.2f} mm{rot} "
            f"(URDF {np.round(r['size_urdf_mm'], 1).tolist()} mm, roadmap {np.round(r['size_roadmap_mm'], 1).tolist()} mm)")


def test_the_fixture_is_a_pinned_export_of_the_four_parity_configurations():
    assert len(DUMP["commit"]) == 40 and all(c in "0123456789abcdef" for c in DUMP["commit"]), "the docs commit is not a full hash"
    assert set(DUMP["sources_sha256"]) == {"docs/assets/roadmap/model/model.js", "docs/assets/roadmap/model/placement.json",
                                           "docs/assets/roadmap/model/finish.json"}
    assert DUMP["head_pitch_deg"] == 30 and DUMP["frame"].startswith("base_footprint")
    stroke = {stop: round(z - STOPS["L1000"], 6) for stop, z in STOPS.items()}
    assert {k: (v["base_plate"], v["lift_joint"]) for k, v in CONFIGS.items()} == {
        "bp000 L1000": ("bp000", stroke["L1000"]), "bp000 L1175": ("bp000", stroke["L1175"]),
        "bp000 L1350": ("bp000", stroke["L1350"]), "bp050 L1000": ("bp050", stroke["L1000"])}
    assert set(DUMP["configs"]) == set(CONFIGS)


def test_the_fixture_version_is_recorded_and_its_confirmation_is_explicit():
    """The parity test proves model == fixture, not model == P-03. The map records which roadmap commit the fixture is and whether the CAD
    owner (Cuma) confirmed it; the record must name the fixture's own commit and source hashes, so a refreshed fixture cannot inherit an old
    confirmation. Unconfirmed is allowed (a WARN, printed on every run); confirmed needs who and when."""
    v = PARITY["version_confirmation"]
    assert v["commit"] == DUMP["commit"] and v["branch"] == DUMP["branch"], "the confirmation record names another roadmap commit than the fixture"
    assert v["sources_sha256"] == DUMP["sources_sha256"], "the confirmation record's source hashes are not the fixture's"
    assert isinstance(v["confirmed"], bool)
    if v["confirmed"]:
        assert v["confirmed_by"] and v["confirmed_on"], "a confirmed fixture names who confirmed it and when"
    else:
        assert v["confirmed_by"] is None and v["confirmed_on"] is None
        warnings.warn(f"roadmap parity fixture {DUMP['commit'][:12]} ({DUMP['branch']}, merged: {DUMP['merged']}) is not confirmed by the "
                      "CAD owner: parity proves model == fixture, not model == P-03", UserWarning)


@pytest.mark.parametrize("config", sorted(CONFIGS))
@pytest.mark.parametrize("item", MAPPED, ids=ITEM_IDS)
def test_every_decided_item_is_within_half_a_millimetre_and_a_fifth_of_a_degree(poses, item, config):
    root, world = poses[config]
    r = compare(item, config, root, world)
    if item.get("pending"):
        assert not r["within"], f"{_describe(item, config, r)}: within tolerance, so it is no longer pending: remove `pending` from the map"
        pytest.skip(f"pending (intended): {item['pending']}; {_describe(item, config, r)}")
    assert r["within"], _describe(item, config, r)


@pytest.mark.parametrize("config", sorted(CONFIGS))
@pytest.mark.parametrize("side", ["left", "right"])
def test_every_openarm_link_frame_is_the_roadmap_rigs(poses, side, config):
    root, world = poses[config]
    rig = DUMP["configs"][config]["arms"][side]
    bad = []
    for entry in PARITY["frames"][side]:
        key = entry["roadmap"]
        flat = rig["base"] if key == "base" else rig["links"][int(key[4:]) - 1] if key.startswith("link") else rig["fingers"][int(key[6:]) - 1]
        page = np.array(flat).reshape(4, 4).T                       # three.js matrices are column-major
        dpos = float(np.linalg.norm(page[:3, 3] - world[entry["urdf"]][:3, 3]))
        drot = _angle_deg(world[entry["urdf"]][:3, :3], page[:3, :3])
        if dpos > TOL_M or drot > TOL_DEG:
            bad.append(f"{entry['urdf']} / {side}:{key}: dpos {dpos * 1000:.2f} mm, drot {drot:.3f} deg")
    assert not bad, f"{config}: " + "; ".join(bad)


def _upper_body_links(root):
    """Links of the upper body: lift_base_plate_link and everything below lift_column_base_link."""
    children = {}
    for j in root.findall("joint"):
        children.setdefault(j.find("parent").get("link"), []).append(j.find("child").get("link"))
    out, todo = {"lift_base_plate_link"}, ["lift_column_base_link"]
    while todo:
        link = todo.pop()
        out.add(link)
        todo += children.get(link, [])
    return out


@pytest.mark.parametrize("base_plate", ["bp000", "bp050"])
def test_every_upper_body_link_with_a_visual_is_mapped_or_listed_with_a_reason(base_plate):
    root = _urdf(base_plate)
    drawn = {l for l in _upper_body_links(root) if root.find(f"link[@name='{l}']/visual") is not None}
    mapped = {l for e in MAPPED for l in e["urdf"]} | {f["urdf"] for side in PARITY["frames"].values() for f in side}
    listed = set(PARITY.get("urdf_only") or {})
    assert drawn - mapped - listed == set(), "upper-body links with a visual and no parity entry"
    assert (mapped | listed) - drawn == set(), "parity entries for links the URDF does not draw"
    assert not mapped & listed


def test_every_upper_body_roadmap_part_is_mapped_or_listed_with_a_reason():
    parts = set(DUMP["configs"]["bp000 L1000"]["parts"])
    mapped = {p.partition("#")[0] for e in MAPPED for p in e["roadmap"]}
    only = PARITY.get("roadmap_only") or {}
    assert parts - mapped - set(only) == set(), "roadmap parts with no parity entry: map them or list them under roadmap_only with a reason"
    assert (mapped | set(only)) - parts == set(), "parity entries for parts the fixture does not have"
    assert not mapped & set(only)
    root = _urdf("bp000")
    links = {l.get("name") for l in root.findall("link")}
    for part, entry in only.items():
        assert entry["attach"] in links and entry["reason"], f"{part}: attach to a URDF link and give the reason"


def test_every_exclusion_names_a_reason_and_matches_a_mesh():
    """A rule that matches nothing is stale; a rule without a reason hides a difference."""
    for rule in PARITY.get("cosmetic", []):
        assert rule["reason"] and any(_excluded(m, [rule]) for c in DUMP["configs"].values() for g in c["parts"].values() for x in g
                                      for m in (x["meshes"] or [])), f"cosmetic rule {rule} matches no mesh"
    for item in MAPPED:
        for rule in item.get("exclude", []):
            groups = roadmap_groups("bp000 L1000", item["roadmap"])
            assert rule["reason"] and any(_excluded(m, [rule]) for g in groups for m in g["meshes"]), f"{item['urdf']}: {rule} matches no mesh"


if __name__ == "__main__":
    rows = {}
    for config, c in CONFIGS.items():
        root = _urdf(c["base_plate"])
        world = zero_pose_poses(root, {"lift_joint": c["lift_joint"]})
        for item in MAPPED:
            r = compare(item, config, root, world)
            rows.setdefault(" + ".join(item["urdf"]), []).append(r)
            print(("ok      " if r["within"] else "PENDING " if item.get("pending") else "OUT     ") + _describe(item, config, r))
    within = sum(all(r["within"] for r in v) for v in rows.values())
    print(f"{len(rows)} mapped items, {within} within tolerance in every configuration, {len(rows) - within} not")
    v = PARITY["version_confirmation"]
    print(f"fixture {DUMP['commit'][:12]} ({DUMP['branch']}): " + (f"confirmed by {v['confirmed_by']} on {v['confirmed_on']}" if v["confirmed"]
          else "NOT confirmed by the CAD owner; parity proves model == fixture, not model == P-03"))
