# SPDX-License-Identifier: MIT
"""The grocery dark store as plain data, from store.yaml. No simulator import: MuJoCo and Isaac Sim composition both read it.

Every element is a dict with world-frame centre `pos` (m), full `size` (m) for boxes or `radius`/`height` for cylinders, `yaw` (rad)
and an rgba colour. Demo input only.
"""
import math
import pathlib
import random

import yaml

HERE = pathlib.Path(__file__).resolve().parent
STORE = HERE / "store.yaml"


def value(entry):
    return entry["value"] if isinstance(entry, dict) and "value" in entry else entry


def load(path=STORE):
    data = yaml.safe_load(pathlib.Path(path).read_text())
    if data.get("schema_version") != 1:
        raise ValueError(f"{path}: schema_version 1 expected")
    return data


def quantities(node, path=""):
    """Every {value, status} entry, as (path, entry)."""
    found = []
    if isinstance(node, dict):
        if "status" in node:
            found.append((path, node))
        for key, child in node.items():
            found += quantities(child, f"{path}.{key}" if path else key)
    elif isinstance(node, list):
        for i, child in enumerate(node):
            found += quantities(child, f"{path}[{i}]")
    return found


SHELF_COLOUR = (0.20, 0.33, 0.55, 1.0)
BOARD_COLOUR = (0.82, 0.83, 0.85, 1.0)
PALETTE = [(0.93, 0.55, 0.10, 1), (0.15, 0.55, 0.25, 1), (0.95, 0.85, 0.25, 1), (0.20, 0.45, 0.80, 1), (0.90, 0.90, 0.88, 1),
           (0.55, 0.25, 0.60, 1), (0.30, 0.70, 0.75, 1), (0.75, 0.35, 0.25, 1)]


def rows(data):
    """{row name: (x of the row centre line, facing sign)}: facing -1 means the front face looks along -x."""
    depth, width = value(data["shelf"]["depth"]), value(data["aisle"]["width"])
    front_r1 = value(data["aisle"]["footprint_front_x"]) + value(data["aisle"]["parking_gap"])
    r1 = front_r1 + depth / 2.0
    r0 = front_r1 - width - depth / 2.0
    r2 = r1 + depth
    r3 = r2 + depth / 2.0 + width + depth / 2.0
    return {"R0": (r0, +1), "R1": (r1, -1), "R2": (r2, +1), "R3": (r3, -1)}


def unit_centres_y(data):
    n, length = data["shelf"]["units_per_row"], value(data["shelf"]["unit_length"])
    return [(i - (n - 1) / 2.0) * length for i in range(n)]


def shelf_boxes(data):
    sh = data["shelf"]
    length, depth = value(sh["unit_length"]), value(sh["depth"])
    board, post = value(sh["board_thickness"]), value(sh["post"])
    levels = [value(level) for level in sh["levels"]]
    height = max(levels)
    boxes = []
    for row, (x, _) in rows(data).items():
        for u, y in enumerate(unit_centres_y(data)):
            for k, top in enumerate(levels):
                boxes.append({"name": f"{row}u{u}_board{k}", "kind": "box", "size": (depth, length, board),
                              "pos": (x, y, top - board / 2.0), "yaw": 0.0, "rgba": BOARD_COLOUR, "role": "shelf"})
            for p, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
                boxes.append({"name": f"{row}u{u}_post{p}", "kind": "box", "size": (post, post, height - board),
                              "pos": (x + sx * (depth - post) / 2.0, y + sy * (length - post) / 2.0, (height - board) / 2.0),
                              "yaw": 0.0, "rgba": SHELF_COLOUR, "role": "shelf"})
    return boxes


def fillers(data):
    """Static filler products on every board except where the pick happens. Deterministic."""
    f = data["fillers"]
    rng = random.Random(f["seed"])
    sh = data["shelf"]
    depth, length, post = value(sh["depth"]), value(sh["unit_length"]), value(sh["post"])
    levels = [value(level) for level in sh["levels"]]
    pitch = value(f["pitch"])
    clear = f["keep_clear"]
    out = []
    for row, (x, facing) in rows(data).items():
        for u, yc in enumerate(unit_centres_y(data)):
            for k, top in enumerate(levels):
                gap = levels[k + 1] - top if k + 1 < len(levels) else 0.45
                y = yc - length / 2.0 + post + 0.06
                n = 0
                while y < yc + length / 2.0 - post - 0.06:
                    cls = rng.choice(f["classes"])
                    h = cls["size"][2] if cls["kind"] == "box" else cls["height"]
                    w = cls["size"][1] if cls["kind"] == "box" else 2 * cls["radius"]
                    dx = cls["size"][0] if cls["kind"] == "box" else 2 * cls["radius"]
                    if h > gap - 0.04:
                        y += pitch / 2
                        continue
                    # front-facing rows of products, one or two deep
                    for r in range(2 if rng.random() < 0.6 else 1):
                        px = x + facing * (depth / 2.0 - dx / 2.0 - 0.01 - r * (dx + 0.01))
                        if row == "R1" and k == 1 and abs(px - clear["centre"][0]) < clear["half"][0] \
                                and abs(y - clear["centre"][1]) < clear["half"][1]:
                            continue
                        colour = PALETTE[rng.randrange(len(PALETTE))]
                        item = {"name": f"{row}u{u}l{k}_{cls['name']}{n}_{r}", "kind": cls["kind"], "pos": (px, y, top + h / 2.0),
                                "yaw": 0.0, "rgba": colour, "role": "filler"}
                        if cls["kind"] == "box":
                            item["size"] = tuple(cls["size"])
                        else:
                            item["radius"], item["height"] = cls["radius"], cls["height"]
                        out.append(item)
                    n += 1
                    y += max(w, pitch * 0.8) + 0.012
    return out


def tote_parts(size, wall):
    """The five boxes of an open tote in its own frame (bottom at z = 0): (name, size, centre)."""
    sx, sy, sz = size
    return [("bottom", (sx, sy, wall), (0, 0, wall / 2.0)),
            ("wall_xn", (wall, sy, sz), (-sx / 2.0 + wall / 2.0, 0, sz / 2.0)),
            ("wall_xp", (wall, sy, sz), (sx / 2.0 - wall / 2.0, 0, sz / 2.0)),
            ("wall_yn", (sx - 2 * wall, wall, sz), (0, -sy / 2.0 + wall / 2.0, sz / 2.0)),
            ("wall_yp", (sx - 2 * wall, wall, sz), (0, sy / 2.0 - wall / 2.0, sz / 2.0))]


def pick_objects(data):
    """The free bodies: the tote on R1 level 0.82 in front of the pick station and the target product in it.

    The tote's long side (0.40 m) runs along the row (y), its 0.30 m side inward (x), as in BUILD1. Returns poses of the body origins:
    tote origin at the centre of its bottom face, product origin at its centre."""
    x1, facing = rows(data)["R1"]
    depth = value(data["shelf"]["depth"])
    front = x1 + facing * depth / 2.0             # the shelf front face, towards the robot
    inward = -facing                               # +x: away from the robot
    t = data["tote"]
    tsize, wall = value(t["size"]), value(t["wall"])
    top = value(data["shelf"]["levels"][t["level"]])
    along, ofs_in = t["offset"]
    tote_x = front + inward * (ofs_in + tsize[1] / 2.0)
    tote_y = data["robot"]["pick_station"]["pose"][1] + along
    tote = {"name": "tote", "size": (tsize[1], tsize[0], tsize[2]), "wall": wall, "mass": value(t["mass"]),
            "pos": (tote_x, tote_y, top), "friction": value(t["friction"])}
    p = data["product"]
    psize = value(p["size"])
    p_along, p_in = p["offset"]
    front_wall = tote_x - inward * tsize[1] / 2.0
    product = {"name": p["name"], "size": tuple(psize), "mass": value(p["mass"]), "friction": value(p["friction"]),
               "pos": (front_wall + inward * p_in, tote_y + p_along, top + wall + psize[2] / 2.0), "rgba": tuple(p["rgba"])}
    return tote, product


def markings(data):
    """Floor tape: aisle edges, the pick station box and the parking spot. Visual only, 1 mm thick."""
    m = data["markings"]
    w, c = m["width"], tuple(m["colour"])
    out = []
    r = rows(data)
    depth = value(data["shelf"]["depth"])
    n, length = data["shelf"]["units_per_row"], value(data["shelf"]["unit_length"])
    span = n * length
    for name, (x, facing) in r.items():
        edge = x + facing * (depth / 2.0 + 0.05)
        out.append({"name": f"tape_{name}", "kind": "box", "size": (w, span, 0.001), "pos": (edge, 0.0, 0.0005), "yaw": 0.0, "rgba": c})

    def frame(name, cx, cy, sx, sy, yaw):
        parts = []
        for k, (dx, dy, ex, ey) in enumerate(((0, sy / 2, sx, w), (0, -sy / 2, sx, w), (sx / 2, 0, w, sy), (-sx / 2, 0, w, sy))):
            c_, s_ = math.cos(yaw), math.sin(yaw)
            parts.append({"name": f"{name}_{k}", "kind": "box", "size": (ex, ey, 0.001),
                          "pos": (cx + c_ * dx - s_ * dy, cy + s_ * dx + c_ * dy, 0.0006), "yaw": yaw, "rgba": c})
        return parts
    px, py, pyaw = data["robot"]["parking_spot"]["pose"]
    out += frame("parking", px, py, 0.95, 0.75, pyaw)
    sx, sy, syaw = data["robot"]["pick_station"]["pose"]
    out += frame("pick_station", sx, sy, 0.95, 0.75, syaw)
    return out


def packing(data):
    pk = data["packing_tote"]
    tx, ty, tyaw = pk["table"]["pose"]
    tsx, tsy, tsz = pk["table"]["size"]
    items = [{"name": "packing_table_top", "kind": "box", "size": (tsx, tsy, 0.04), "pos": (tx, ty, tsz - 0.02), "yaw": tyaw,
              "rgba": (0.55, 0.42, 0.30, 1)}]
    for k, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        items.append({"name": f"packing_table_leg{k}", "kind": "box", "size": (0.05, 0.05, tsz - 0.04),
                      "pos": (tx + sx * (tsx / 2 - 0.05), ty + sy * (tsy / 2 - 0.05), (tsz - 0.04) / 2), "yaw": tyaw,
                      "rgba": (0.25, 0.25, 0.27, 1)})
    size, wall = value(pk["size"]), value(pk["wall"])
    px, py, _ = pk["pose"]
    for name, s, c in tote_parts(size, wall):
        items.append({"name": f"packing_tote_{name}", "kind": "box", "size": s, "pos": (px + c[0], py + c[1], tsz + c[2]),
                      "yaw": 0.0, "rgba": (0.10, 0.45, 0.75, 1)})
    return items


def layout(data=None):
    data = data or load()
    tote, product = pick_objects(data)
    return {"shelves": shelf_boxes(data), "fillers": fillers(data), "markings": markings(data), "packing": packing(data),
            "tote": tote, "product": product, "lights": data["lights"], "data": data}
