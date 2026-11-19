# SPDX-License-Identifier: MIT
"""The grocery dark store demonstrator scene: what config/grocery_scene.yaml adds to the Build 1 scene model.

scene.py calls this module only for a scene with a `grocery` section (scene.extension). It adds fixed boxes (aisle
shelving with plinths and back panels, decorative boards and post extensions of the pick unit, the packing bench, the
order tote and the store walls), the fixture colours, and visual-only world elements (filler products, floor markings,
a fill light). Everything that a box adds also reaches the Nav2 map (where it cuts the LiDAR plane) and the MoveIt
planning scene, because both are built from scene.static_boxes. The filler products and markings are visual only:
no collision, not in the map, not in the planning scene.

The pick unit (`work_shelf`), its tray (`tote`), the target product (`item`) and the `payload` are the Baseline 1
objects, unchanged relative to the work-shelf station. The packing station keeps the hand's pose relative to the robot:
the order tote stands where the held product arrives when the robot parks at the packing bench the way it parked at the
work shelf (packing_gap instead of parking_gap). No ROS import. Simulation input only.
"""
import math

from openamrobot_build1_scene import scene as scene_model

value = scene_model.value

PALETTE = ("0.85 0.20 0.15 1", "0.95 0.75 0.10 1", "0.20 0.55 0.25 1", "0.15 0.35 0.75 1", "0.95 0.95 0.92 1",
           "0.90 0.45 0.10 1", "0.55 0.25 0.60 1", "0.35 0.70 0.80 1", "0.60 0.40 0.25 1", "0.80 0.80 0.30 1")
COLOURS = {"plinth": "0.25 0.27 0.30 1", "back": "0.82 0.84 0.86 1", "board": "0.62 0.64 0.67 1",
           "post": "0.35 0.37 0.40 1", "bench": "0.70 0.72 0.74 1", "order_tote": "0.10 0.45 0.35 1",
           "wall": "0.90 0.90 0.86 1", "deco": "0.62 0.64 0.67 1", "bollard": "0.95 0.75 0.05 1"}


def _rot(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x - s * y, s * x + c * y


def _place(local, pose):
    x, y = _rot(local[0], local[1], pose[2])
    return pose[0] + x, pose[1] + y, local[2]


def _hash(*parts):
    """A deterministic integer from `parts` (no global random state, the same on every machine)."""
    h = 2166136261
    for part in parts:
        for ch in str(part):
            h = ((h ^ ord(ch)) * 16777619) & 0xFFFFFFFF
    return h


# ---------------------------------------------------------------- the aisles

def row_units(scene):
    """{unit name: (x, y, yaw)} of every aisle unit (the pick unit is skipped: it is the work shelf of scene.yaml)."""
    length = value(scene["shelf"]["unit_length"])
    units = {}
    for row in scene["grocery"]["rows"]:
        x0, y0 = row["start"]
        dx, dy = _rot(1.0, 0.0, row["run_yaw"])
        for index in range(row["units"]):
            if index in row.get("skip", []):
                continue
            along = (index + 0.5) * length
            units[f"{row['name']}_{index}"] = (x0 + dx * along, y0 + dy * along, row["yaw"])
    return units


def _unit_boxes(scene, name, pose, plinth=True, back=True):
    """Boards, posts, plinth and back panel of one aisle unit, as scene.shelf_boxes: local x along the unit, local y
    from the back to the front, the front face at local y = +depth / 2."""
    shelf, grocery = scene["shelf"], scene["grocery"]
    length, depth = value(shelf["unit_length"]), value(shelf["depth"])
    board, post = value(shelf["board_thickness"]), value(shelf["post"])
    levels = [value(level) for level in grocery["row_levels"]]
    boxes = [(f"{name}_board_{i}", (length, depth, board), (0.0, 0.0, top - board / 2.0)) for i, top in enumerate(levels)]
    height = max(levels)
    for i, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        boxes.append((f"{name}_post_{i}", (post, post, height - board),
                      (sx * (length - post) / 2.0, sy * (depth - post) / 2.0, (height - board) / 2.0)))
    if plinth:
        p = grocery["plinth"]
        h, recess = value(p["height"]), value(p["recess"])
        # Between the posts, from the rear posts to `recess` behind the front edge: a closed line in the LiDAR plane.
        boxes.append((f"{name}_plinth", (length - 2 * post, depth - recess - post, h), (0.0, (post - recess) / 2.0, h / 2.0)))
    if back:
        t = value(grocery["back_panel"]["thickness"])
        bottom = levels[0]
        boxes.append((f"{name}_back", (length - 2 * post, t, height - bottom),
                      (0.0, -depth / 2.0 + t / 2.0, bottom + (height - bottom) / 2.0)))
    return [(box, size, _place(centre, pose), pose[2]) for box, size, centre in boxes]


def _work_shelf_decoration(scene):
    """Decorative boards of the pick unit: the levels of row_levels that the Baseline 1 work shelf does not have, and
    its posts extended to the top board. Its own two boards, posts and the open space under level 0 are unchanged."""
    shelf, grocery = scene["shelf"], scene["grocery"]
    length, depth = value(shelf["unit_length"]), value(shelf["depth"])
    board, post = value(shelf["board_thickness"]), value(shelf["post"])
    own = [value(level) for level in shelf["levels"]]
    pose = tuple(scene["work_shelf"]["pose"])
    boxes = []
    for i, top in enumerate(value(level) for level in grocery["row_levels"]):
        if any(abs(top - level) < 1e-6 for level in own):
            continue
        boxes.append((f"work_shelf_deco_board_{i}", (length, depth, board), (0.0, 0.0, top - board / 2.0)))
    low, high = max(own) - board, max(value(level) for level in grocery["row_levels"]) - board
    if high > low:
        for i, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
            boxes.append((f"work_shelf_deco_post_{i}", (post, post, high - low),
                          (sx * (length - post) / 2.0, sy * (depth - post) / 2.0, low + (high - low) / 2.0)))
    return [(box, size, _place(centre, pose), pose[2]) for box, size, centre in boxes]


# ---------------------------------------------------------------- packing bench, order tote, walls

def bench_pose(scene):
    return tuple(scene["grocery"]["packing_bench"]["pose"])


def bench_front(scene):
    """(point on the bench front edge at its centre, outward unit normal), in the world."""
    x, y, yaw = bench_pose(scene)
    depth = value(scene["grocery"]["packing_bench"]["size"])[1]
    nx, ny = _rot(0.0, 1.0, yaw)
    return (x + nx * depth / 2.0, y + ny * depth / 2.0), (nx, ny)


def station(scene, name):
    """(x, y, yaw) of base_footprint at station `name`: `packing` here, every other name from scene.station."""
    if name != "packing":
        return scene_model.station(scene, name)
    (fx, fy), (nx, ny) = bench_front(scene)
    gap = value(scene["grocery"]["packing_bench"]["packing_gap"]) + scene_model.FOOTPRINT_FRONT_X
    return fx + nx * gap, fy + ny * gap, math.atan2(-ny, -nx)


def _to_station(point, pose):
    """A world point (x, y) in the frame of a station pose (x, y, yaw)."""
    return _rot(point[0] - pose[0], point[1] - pose[1], -pose[2])


def _from_station(point, pose):
    x, y = _rot(point[0], point[1], pose[2])
    return pose[0] + x, pose[1] + y


def drop_point(scene):
    """(x, y) where the held product arrives over the bench: the product's place relative to the work-shelf station,
    carried to the packing station. The arm does not move between the two stations."""
    item = scene_model.movable_objects(scene)["item"]["pose"]
    local = _to_station(item[:2], scene_model.station(scene, "work_shelf"))
    return _from_station(local, station(scene, "packing"))


def order_tote(scene):
    """{centre (x, y), yaw, size (along, deep, height), wall, floor_top z, wall_top z, inner half sizes (along, deep)}."""
    spec = scene["grocery"]["order_tote"]
    size, wall = value(spec["size"]), value(spec["wall"])
    top = value(scene["grocery"]["packing_bench"]["height"])
    cx, cy = drop_point(scene)
    return {"centre": (cx, cy), "yaw": bench_pose(scene)[2], "size": tuple(size), "wall": wall, "bench_top": top,
            "floor_top": top + wall, "wall_top": top + size[2], "inner_half": (size[0] / 2.0 - wall, size[1] / 2.0 - wall)}


def in_tote(scene, point):
    """(along, deep) of a world point in the order tote frame (centre, bench yaw)."""
    tote = order_tote(scene)
    return _rot(point[0] - tote["centre"][0], point[1] - tote["centre"][1], -tote["yaw"])


def _bench_boxes(scene):
    bench = scene["grocery"]["packing_bench"]
    length, depth = value(bench["size"])
    top, thick, leg = value(bench["height"]), value(bench["top_thickness"]), value(bench["leg"])
    pose = bench_pose(scene)
    boxes = [("packing_bench_top", (length, depth, thick), (0.0, 0.0, top - thick / 2.0))]
    for i, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        boxes.append((f"packing_bench_leg_{i}", (leg, leg, top - thick),
                      (sx * (length - leg) / 2.0, sy * (depth - leg) / 2.0, (top - thick) / 2.0)))
    out = [(name, size, _place(centre, pose), pose[2]) for name, size, centre in boxes]
    tote = order_tote(scene)
    (sa, sd, sh), wall, base = tote["size"], tote["wall"], tote["bench_top"]
    parts = [("floor", (sa, sd, wall), (0.0, 0.0, base + wall / 2.0)),
             ("front", (sa, wall, sh - wall), (0.0, sd / 2.0 - wall / 2.0, base + wall + (sh - wall) / 2.0)),
             ("back", (sa, wall, sh - wall), (0.0, -sd / 2.0 + wall / 2.0, base + wall + (sh - wall) / 2.0)),
             ("left", (wall, sd - 2 * wall, sh - wall), (-sa / 2.0 + wall / 2.0, 0.0, base + wall + (sh - wall) / 2.0)),
             ("right", (wall, sd - 2 * wall, sh - wall), (sa / 2.0 - wall / 2.0, 0.0, base + wall + (sh - wall) / 2.0))]
    tote_pose = (tote["centre"][0], tote["centre"][1], tote["yaw"])
    out += [(f"order_tote_{name}", size, _place(centre, tote_pose), tote["yaw"]) for name, size, centre in parts]
    return out


def _walls(scene):
    fx, fy = value(scene["floor"]["size"])
    spec = scene["grocery"]["walls"]
    t, h = value(spec["thickness"]), value(spec["height"])
    return [("wall_south", (fx, t, h), (0.0, -fy / 2.0 + t / 2.0, h / 2.0), 0.0),
            ("wall_north", (fx, t, h), (0.0, fy / 2.0 - t / 2.0, h / 2.0), 0.0),
            ("wall_west", (t, fy - 2 * t, h), (-fx / 2.0 + t / 2.0, 0.0, h / 2.0), 0.0),
            ("wall_east", (t, fy - 2 * t, h), (fx / 2.0 - t / 2.0, 0.0, h / 2.0), 0.0)]


def _bollards(scene):
    spec = scene["grocery"].get("bollards")
    if not spec:
        return []
    sx, sy, sz = value(spec["size"])
    offset = value(spec["offset"])
    west, east = spec["row_ends_x"]
    return [(f"bollard_{i}_{j}", (sx, sy, sz), (x, y, sz / 2.0), 0.0)
            for i, y in enumerate(spec["row_lines_y"]) for j, x in enumerate((west - offset, east + offset))]


def static_boxes(scene):
    """The fixed boxes this extension adds to scene.static_boxes: (name, size, centre, yaw)."""
    boxes = []
    for name, pose in row_units(scene).items():
        row = next(r for r in scene["grocery"]["rows"] if name.startswith(r["name"] + "_"))
        boxes += _unit_boxes(scene, name, pose, plinth=True, back=row.get("back_panel", True))
    boxes += _work_shelf_decoration(scene)
    boxes += _bench_boxes(scene)
    boxes += _bollards(scene)
    boxes += _walls(scene)
    return boxes


def colour(name):
    """Ambient and diffuse colour of a fixed box, from its name. The Baseline 1 shelf colour where none matches."""
    for prefix, key in (("order_tote", "order_tote"), ("packing_bench", "bench"), ("wall_", "wall"), ("work_shelf_deco", "deco"),
                        ("bollard_", "bollard")):
        if name.startswith(prefix):
            return COLOURS[key]
    for key in ("plinth", "back", "post", "board"):
        if name.endswith(f"_{key}") or f"_{key}_" in name:
            return COLOURS[key]
    return "0.55 0.45 0.3 1"


# ---------------------------------------------------------------- visual-only world elements

def products(scene):
    """Filler products, visual only: (name, size, centre, yaw, colour). On the aisle boards and on the boards of the
    pick unit that the arm does not use (not on level 0 of the work shelf, not next to the payload on level 1)."""
    spec = scene["grocery"]["products"]
    shelf = scene["shelf"]
    length, depth, post = value(shelf["unit_length"]), value(shelf["depth"]), value(shelf["post"])
    levels = [value(level) for level in scene["grocery"]["row_levels"]]
    kinds = spec["kinds"]
    units = dict(row_units(scene))
    units["work_shelf"] = tuple(scene["work_shelf"]["pose"])
    own = [value(level) for level in shelf["levels"]]
    px, py, _pz, _yaw = scene_model.movable_objects(scene)["payload"]["pose"]
    payload_x = _to_station((px, py), units["work_shelf"])[0]      # along the work shelf, from its centre
    payload_half = value(scene["payload"]["size"])[0] / 2.0 + 0.05
    out = []
    for unit, pose in sorted(units.items()):
        for li, top in enumerate(levels):
            if unit == "work_shelf" and abs(top - own[0]) < 1e-6:
                continue                     # level 0 of the pick unit: the tray and the target product only
            x = -length / 2.0 + post + 0.02
            n = 0
            while True:
                h = _hash(unit, li, n)
                kind = kinds[h % len(kinds)]
                sx, sy, sz = kind["size"]
                if x + sx > length / 2.0 - post - 0.02:
                    break
                if unit == "work_shelf" and abs(top - own[1]) < 1e-6 and abs(x + sx / 2.0 - payload_x) < payload_half + sx / 2.0:
                    x += sx + 0.03           # keep clear of the payload on level 1
                    n += 1
                    continue
                cx, cy = x + sx / 2.0, depth / 2.0 - 0.03 - sy / 2.0
                for k in range(spec["deep"]):
                    centre = (cx, cy - k * (sy + 0.01), top + sz / 2.0)
                    if centre[1] - sy / 2.0 < -depth / 2.0 + 0.03:
                        break
                    out.append((f"product_{unit}_{li}_{n}_{k}", (sx, sy, sz), _place(centre, pose), pose[2],
                                PALETTE[(h >> 4) % len(PALETTE)]))
                x += sx + spec["gap"]
                n += 1
    return out


def markings(scene):
    """Floor markings, visual only: (name, size, centre, yaw, colour). The parking bay and the packing zone."""
    spec = scene["grocery"]["markings"]
    w, t = value(spec["line_width"]), 0.002
    out = []
    for label, (x, y, yaw), (lx, ly) in (("parking", scene_model.station(scene, "origin"), spec["parking_bay"]),
                                         ("packing", station(scene, "packing"), spec["packing_bay"])):
        for i, (dx, dy, sx, sy) in enumerate(((0.0, ly / 2.0, lx, w), (0.0, -ly / 2.0, lx, w),
                                              (lx / 2.0, 0.0, w, ly), (-lx / 2.0, 0.0, w, ly))):
            px, py = _rot(dx, dy, yaw)
            out.append((f"marking_{label}_{i}", (sx, sy, t), (x + px, y + py, t / 2.0 + 0.0005), yaw, spec["colour"]))
    return out


def _visual(name, size, centre, yaw, colour_text):
    geometry = f"<geometry><box><size>{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}</size></box></geometry>"
    return (f'<visual name="{name}"><pose>{centre[0]:.4f} {centre[1]:.4f} {centre[2]:.4f} 0 0 {yaw:.5f}</pose>{geometry}'
            f"<material><ambient>{colour_text}</ambient><diffuse>{colour_text}</diffuse></material></visual>")


def world_elements(scene):
    """SDF elements appended to the world: one static model of visual-only products and markings, and a fill light."""
    visuals = [_visual(*entry) for entry in products(scene) + markings(scene)]
    out = ['<model name="store_decor"><static>true</static><link name="link">' + "".join(visuals) + "</link></model>"]
    out.append('<light type="directional" name="store_fill"><cast_shadows>false</cast_shadows><pose>0 0 8 0 0 0</pose>'
               "<diffuse>0.45 0.45 0.45 1</diffuse><specular>0.05 0.05 0.05 1</specular><direction>0.3 -0.5 -0.8</direction></light>")
    return out
