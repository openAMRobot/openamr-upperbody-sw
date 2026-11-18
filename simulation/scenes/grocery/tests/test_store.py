# SPDX-License-Identifier: MIT
"""The store layout: provenance of every number, the product of the Gazebo demo, geometry that does not overlap. No inputs needed."""
import itertools

import pytest

import store as st


@pytest.fixture(scope="module")
def data():
    return st.load()


@pytest.fixture(scope="module")
def layout(data):
    return st.layout(data)


def test_every_number_has_a_status_and_copied_ones_a_source(data):
    found = st.quantities(data)
    assert len(found) > 15
    for path, entry in found:
        assert entry["status"] in ("copied", "estimated"), path
        if entry["status"] == "copied":
            assert entry.get("source"), path


def test_product_matches_the_build1_gazebo_item(data, layout):
    """Build 1 scene.yaml (openamr-upperbody-sw PR 73 head 1a84066): item 0.035 x 0.035 x 0.14 m, 0.15 kg, friction 1.0;
    tote 0.40 x 0.30 x 0.06 m, wall 5 mm, 0.6 kg; levels 0.82 and 1.09 m; aisle 1.50 m; parking gap 0.30 m."""
    p = layout["product"]
    assert p["size"] == pytest.approx((0.035, 0.035, 0.14)) and p["mass"] == pytest.approx(0.15) and p["friction"] == 1.0
    t = layout["tote"]
    assert sorted(t["size"]) == pytest.approx(sorted((0.40, 0.30, 0.06))) and t["wall"] == 0.005 and t["mass"] == 0.6
    levels = [st.value(v) for v in data["shelf"]["levels"]]
    assert 0.82 in levels and 1.09 in levels
    assert st.value(data["aisle"]["width"]) == 1.50 and st.value(data["aisle"]["parking_gap"]) == 0.30
    assert st.value(data["pick"]["lift_speed"]) == 0.018


def _aabb(e):
    if e["kind"] == "box":
        s = e["size"]
    else:
        s = (2 * e["radius"], 2 * e["radius"], e["height"])
    assert not e.get("yaw"), e["name"]
    return [(e["pos"][i] - s[i] / 2, e["pos"][i] + s[i] / 2) for i in range(3)]


def _overlap(a, b, eps=1e-6):
    return all(a[k][0] < b[k][1] - eps and b[k][0] < a[k][1] - eps for k in range(3))


def test_fillers_overlap_nothing(layout):
    fillers = [_aabb(e) for e in layout["fillers"]]
    shelves = [_aabb(e) for e in layout["shelves"]]
    for i, j in itertools.combinations(range(len(fillers)), 2):
        assert not _overlap(fillers[i], fillers[j]), (layout["fillers"][i]["name"], layout["fillers"][j]["name"])
    for i, f in enumerate(fillers):
        assert not any(_overlap(f, s) for s in shelves), layout["fillers"][i]["name"]
    assert len(fillers) > 300


def test_product_stands_in_the_tote_on_the_pick_shelf(data, layout):
    t, p = layout["tote"], layout["product"]
    x1, facing = st.rows(data)["R1"]
    front = x1 + facing * st.value(data["shelf"]["depth"]) / 2
    assert t["pos"][2] == pytest.approx(0.82)
    assert t["pos"][0] - t["size"][0] / 2 == pytest.approx(front)           # tote front edge on the shelf front
    assert p["pos"][2] - p["size"][2] / 2 == pytest.approx(0.82 + t["wall"])  # on the tote bottom
    inner_x = (t["pos"][0] - t["size"][0] / 2 + t["wall"], t["pos"][0] + t["size"][0] / 2 - t["wall"])
    inner_y = (t["pos"][1] - t["size"][1] / 2 + t["wall"], t["pos"][1] + t["size"][1] / 2 - t["wall"])
    assert inner_x[0] < p["pos"][0] - p["size"][0] / 2 and p["pos"][0] + p["size"][0] / 2 < inner_x[1]
    assert inner_y[0] < p["pos"][1] - p["size"][1] / 2 and p["pos"][1] + p["size"][1] / 2 < inner_y[1]
    assert p["pos"][1] == pytest.approx(0.16)                                # on the robot's left, as in Build 1
    tote_box = [(t["pos"][i] - t["size"][i] / 2, t["pos"][i] + t["size"][i] / 2) for i in range(2)] + [(0.82, 0.88)]
    assert not any(_overlap(tote_box, _aabb(f)) for f in layout["fillers"])


def test_aisles_and_the_pick_station_are_clear(data, layout):
    rows = st.rows(data)
    depth = st.value(data["shelf"]["depth"])
    faces = {name: x + facing * depth / 2 for name, (x, facing) in rows.items()}
    assert faces["R1"] - faces["R0"] == pytest.approx(1.50)
    assert faces["R3"] - faces["R2"] == pytest.approx(1.50)
    # the platform-sw footprint front at the pick station is the parking gap from the shelf front
    assert faces["R1"] - st.value(data["aisle"]["footprint_front_x"]) == pytest.approx(0.30)
    # a 0.80 x 0.65 m envelope (UC1 limits, Build 1) at the pick station and along aisle 1 touches no shelf or filler
    for y in (-3.2, -2.0, -1.0, 0.0):
        env = [(-0.40, 0.40), (y - 0.325, y + 0.325), (0.0, 1.5)]
        for e in layout["shelves"] + layout["fillers"] + layout["packing"]:
            assert not _overlap(env, _aabb(e)), (y, e["name"])


def test_layout_is_deterministic(data):
    assert st.layout(data)["fillers"] == st.layout(st.load())["fillers"]
