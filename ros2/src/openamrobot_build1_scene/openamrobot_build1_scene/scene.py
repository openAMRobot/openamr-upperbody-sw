# SPDX-License-Identifier: MIT
"""The Build 1 scene as data: static boxes, movable objects, stations, the Gazebo world and the Nav2 map.

config/scene.yaml is the only source. This module has no ROS import, so the tests run anywhere.
Simulation input only.
"""
import math
import os

import yaml

STATUSES = ("sourced", "derived", "estimated")
# The octagon of openamr-platform-sw nav2_params.yaml at 0741778. Its front edge is at x = +0.415 m.
FOOTPRINT_FRONT_X = 0.415


def load(path):
    with open(path, encoding="utf-8") as f:
        scene = yaml.safe_load(f)
    if scene.get("schema_version") != 1:
        raise ValueError("scene.yaml: schema_version 1 expected")
    return scene


def value(entry):
    """The number behind a {value, status, source} entry, or the entry itself."""
    return entry["value"] if isinstance(entry, dict) and "value" in entry else entry


def quantities(node, path=""):
    """Every {value, status, ...} entry of the scene, as (path, entry)."""
    found = []
    if isinstance(node, dict):
        if "status" in node:
            found.append((path, node))
        for key, child in node.items():
            found += quantities(child, f"{path}.{key}" if path else key)
    elif isinstance(node, list):
        for index, child in enumerate(node):
            found += quantities(child, f"{path}[{index}]")
    return found


def _rot(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x - s * y, s * x + c * y


def _place(local_xyz, pose):
    """A point of a model at `pose` (x, y, yaw) in the world."""
    x, y = _rot(local_xyz[0], local_xyz[1], pose[2])
    return pose[0] + x, pose[1] + y, local_xyz[2]


def shelf_boxes(scene, name, pose):
    """The boxes of one shelf unit: (name, size xyz, centre xyz, yaw). Local frame: x along the unit,
    y from the back to the front, z up from the floor. The front face is at local y = +depth / 2."""
    shelf = scene["shelf"]
    length, depth = value(shelf["unit_length"]), value(shelf["depth"])
    board, post = value(shelf["board_thickness"]), value(shelf["post"])
    levels = [value(level) for level in shelf["levels"]]
    boxes = []
    for index, top in enumerate(levels):
        boxes.append((f"{name}_board_{index}", (length, depth, board), (0.0, 0.0, top - board / 2.0)))
    height = max(levels)
    for index, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        boxes.append((f"{name}_post_{index}", (post, post, height - board),
                      (sx * (length - post) / 2.0, sy * (depth - post) / 2.0, (height - board) / 2.0)))
    return [(box_name, size, _place(centre, pose), pose[2]) for box_name, size, centre in boxes]


def shelf_poses(scene):
    """{unit name: (x, y, yaw)} of every shelf unit: the two aisle rows and the work shelf."""
    aisle, shelf = scene["aisle"], scene["shelf"]
    length, depth, width = value(shelf["unit_length"]), value(shelf["depth"]), value(aisle["width"])
    x0, y0, yaw = aisle["centre"]
    poses = {}
    for unit in range(shelf["units_per_side"]):
        along = (unit + 0.5) * length
        for side, sign in (("a", 1.0), ("b", -1.0)):
            dx, dy = _rot(along, sign * (width + depth) / 2.0, yaw)
            # Side a is on +y of the centre line and faces -y. Side b faces +y.
            poses[f"shelf_{side}{unit}"] = (x0 + dx, y0 + dy, yaw + (math.pi if sign > 0 else 0.0))
    poses["work_shelf"] = tuple(scene["work_shelf"]["pose"])
    return poses


def extension(scene):
    """The module that adds fixtures to a scene with an extension section, or None. Only the grocery dark store demo
    (config/grocery_scene.yaml, key `grocery`) has one; config/scene.yaml has none and is unchanged by this."""
    if "grocery" in scene:
        from openamrobot_build1_scene import grocery_scene
        return grocery_scene
    return None


def static_boxes(scene):
    """Every fixed box of the scene: (name, size, centre, yaw). Cylinders are in obstacles()."""
    boxes = []
    for name, pose in shelf_poses(scene).items():
        boxes += shelf_boxes(scene, name, pose)
    for obstacle in scene["obstacles"]:
        if obstacle["shape"] == "box":
            x, y, yaw = obstacle["pose"]
            boxes.append((obstacle["name"], tuple(obstacle["size"]), (x, y, obstacle["size"][2] / 2.0), yaw))
    if extension(scene) is not None:
        boxes += extension(scene).static_boxes(scene)
    return boxes


def cylinders(scene):
    return [(o["name"], o["radius"], o["height"], (o["pose"][0], o["pose"][1], o["height"] / 2.0))
            for o in scene["obstacles"] if o["shape"] == "cylinder"]


def work_shelf_front(scene):
    """(point on the front edge at the unit centre, outward unit vector) of the work shelf, in the world."""
    x, y, yaw = scene["work_shelf"]["pose"]
    depth = value(scene["shelf"]["depth"])
    nx, ny = _rot(0.0, 1.0, yaw)
    return (x + nx * depth / 2.0, y + ny * depth / 2.0), (nx, ny)


def station_marker_pose(scene):
    """(x, y, z, yaw) of the station marker model: its +x face (the tags) along the shelf's outward normal, the face
    `face_offset` in front of the shelf front edge (model origin half the plate thickness behind the face)."""
    (fx, fy), (nx, ny) = work_shelf_front(scene)
    marker = scene["station_marker"]
    offset = value(marker["face_offset"])
    return fx + nx * offset / 2.0, fy + ny * offset / 2.0, value(marker["height"]), math.atan2(ny, nx)


def station_marker_standoff(scene):
    """Distance from the marker face to the work-shelf station along the face normal (the detector's `standoff`)."""
    return value(scene["work_shelf"]["parking_gap"]) + FOOTPRINT_FRONT_X - value(scene["station_marker"]["face_offset"])


def station(scene, name):
    """(x, y, yaw) where the robot's base_footprint is spawned for station `name`."""
    entry = scene["stations"][name]
    if "pose" in entry:
        return tuple(entry["pose"])
    (fx, fy), (nx, ny) = work_shelf_front(scene)
    gap = value(scene["work_shelf"]["parking_gap"]) + FOOTPRINT_FRONT_X
    return fx + nx * gap, fy + ny * gap, math.atan2(-ny, -nx)


def movable_objects(scene):
    """The free bodies: {name: {size, mass, pose (x, y, z, yaw), friction, kind}}. z is the centre height."""
    (fx, fy), (nx, ny) = work_shelf_front(scene)
    tx, ty = -ny, nx                       # along the shelf, to the left when you look at its front
    yaw = scene["work_shelf"]["pose"][2]
    levels = [value(level) for level in scene["shelf"]["levels"]]
    tote, item, payload = scene["tote"], scene["item"], scene["payload"]
    tote_size, wall = value(tote["size"]), value(tote["wall"])
    along, inward = tote["offset"]
    tote_x = fx + tx * along - nx * (inward + tote_size[1] / 2.0)
    tote_y = fy + ty * along - ny * (inward + tote_size[1] / 2.0)
    tote_z = levels[tote["level"]]
    item_size = value(item["size"])
    item_along, item_inward = item["offset"]
    front_x = tote_x + nx * tote_size[1] / 2.0
    front_y = tote_y + ny * tote_size[1] / 2.0
    item_x = front_x + tx * item_along - nx * item_inward
    item_y = front_y + ty * item_along - ny * item_inward
    payload_size = value(payload["size"])
    return {
        "tote": {"kind": "tote", "size": tuple(tote_size), "wall": wall, "mass": value(tote["mass"]),
                 "pose": (tote_x, tote_y, tote_z, yaw), "friction": 0.8},
        "item": {"kind": "box", "size": tuple(item_size), "mass": value(item["mass"]),
                 "pose": (item_x, item_y, tote_z + wall + item_size[2] / 2.0, yaw), "friction": value(item["friction"])},
        "payload": {"kind": "box", "size": tuple(payload_size), "mass": value(payload["mass"]),
                    "pose": (fx - tx * 0.35 - nx * 0.15, fy - ty * 0.35 - ny * 0.15,
                             levels[payload["level"]] + payload_size[2] / 2.0, yaw), "friction": 0.8},
    }


def shelf_reach_targets(scene):
    """{level index: {left, right}} world points the arm scenario reaches for: above the front edge of each
    work shelf level, 0.16 m to each side of the unit centre. `clearance` is the height above the board."""
    (fx, fy), (nx, ny) = work_shelf_front(scene)
    tx, ty = -ny, nx
    targets = {}
    for index, level in enumerate(scene["shelf"]["levels"]):
        z = value(level) + 0.16
        # The robot faces the shelf, so its left is the shelf's -t side.
        targets[index] = {"left": (fx - tx * 0.16 + nx * 0.05, fy - ty * 0.16 + ny * 0.05, z),
                          "right": (fx + tx * 0.16 + nx * 0.05, fy + ty * 0.16 + ny * 0.05, z)}
    return targets


# ---------------------------------------------------------------- Gazebo world

def _box_inertia(mass, size):
    x, y, z = size
    return mass * (y * y + z * z) / 12.0, mass * (x * x + z * z) / 12.0, mass * (x * x + y * y) / 12.0


def _surface(mu):
    return (f"<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>")


def _box_link_parts(name, size, pose_xyz, mu, colour):
    sx, sy, sz = size
    px, py, pz = pose_xyz
    geometry = f"<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
    return (f'<collision name="{name}_collision"><pose>{px} {py} {pz} 0 0 0</pose>{geometry}{_surface(mu)}</collision>'
            f'<visual name="{name}_visual"><pose>{px} {py} {pz} 0 0 0</pose>{geometry}'
            f"<material><ambient>{colour}</ambient><diffuse>{colour}</diffuse></material></visual>")


POSE_PUBLISHER = ('<plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">'
                  "<publish_model_pose>true</publish_model_pose><publish_link_pose>false</publish_link_pose>"
                  "<publish_nested_model_pose>false</publish_nested_model_pose>"
                  "<use_pose_vector_msg>false</use_pose_vector_msg><update_frequency>50</update_frequency></plugin>")


def _movable_sdf(name, body):
    x, y, z, yaw = body["pose"]
    mass = body["mass"]
    if body["kind"] == "tote":
        sx, sy, sz = body["size"]
        wall = body["wall"]
        parts = [("bottom", (sx, sy, wall), (0, 0, wall / 2.0)),
                 ("front", (sx, wall, sz), (0, sy / 2.0 - wall / 2.0, sz / 2.0)),
                 ("back", (sx, wall, sz), (0, -sy / 2.0 + wall / 2.0, sz / 2.0)),
                 ("left", (wall, sy, sz), (-sx / 2.0 + wall / 2.0, 0, sz / 2.0)),
                 ("right", (wall, sy, sz), (sx / 2.0 - wall / 2.0, 0, sz / 2.0))]
        shapes = "".join(_box_link_parts(part, size, centre, body["friction"], "0.1 0.3 0.7 1") for part, size, centre in parts)
        inertia = _box_inertia(mass, body["size"])
        com = f"0 0 {sz / 3.0}"
    else:
        shapes = _box_link_parts("body", body["size"], (0, 0, 0), body["friction"], "0.8 0.5 0.1 1")
        inertia = _box_inertia(mass, body["size"])
        com = "0 0 0"
    return (f'<model name="{name}"><pose>{x} {y} {z} 0 0 {yaw}</pose><link name="link">'
            f"<inertial><pose>{com} 0 0 0</pose><mass>{mass}</mass><inertia><ixx>{inertia[0]:.6g}</ixx><iyy>{inertia[1]:.6g}</iyy>"
            f"<izz>{inertia[2]:.6g}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>{shapes}</link>{POSE_PUBLISHER}</model>")


def world_sdf(scene, profile, sensors=True, station_marker=False):
    """The world as SDF 1.9 text, for physics profile `profile` (fine or fast)."""
    physics = scene["physics_profiles"][profile]
    floor = scene["floor"]
    fx, fy = value(floor["size"])
    mu = value(floor["friction"]["mu"])
    out = ['<?xml version="1.0"?>',
           "<!-- SPDX-License-Identifier: MIT -->",
           "<!-- Generated from config/scene.yaml by openamrobot_build1_scene. Do not edit. Simulation only. -->",
           '<sdf version="1.9">', f'<world name="{scene["world_name"]}">',
           f'<physics name="{profile}" type="ignored"><max_step_size>{physics["max_step_size"]}</max_step_size>'
           f'<real_time_factor>{physics["real_time_factor"]}</real_time_factor></physics>',
           '<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>',
           '<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>',
           '<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>']
    if sensors:
        out.append('<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">'
                   "<render_engine>ogre2</render_engine></plugin>")
    out.append('<light type="directional" name="sun"><cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose>'
               "<diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.4 0.2 -0.9</direction></light>")
    out.append('<model name="floor"><static>true</static><link name="link">'
               '<collision name="collision"><geometry><plane><normal>0 0 1</normal>'
               f"<size>{fx} {fy}</size></plane></geometry>{_surface(mu)}</collision>"
               f'<visual name="visual"><geometry><plane><normal>0 0 1</normal><size>{fx} {fy}</size></plane></geometry>'
               "<material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material></visual></link></model>")
    out.append('<model name="fixtures"><static>true</static><link name="link">')
    for name, size, centre, yaw in static_boxes(scene):
        geometry = f"<geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry>"
        pose = f"<pose>{centre[0]:.5f} {centre[1]:.5f} {centre[2]:.5f} 0 0 {yaw:.5f}</pose>"
        colour = extension(scene).colour(name) if extension(scene) is not None else "0.55 0.45 0.3 1"
        out.append(f'<collision name="{name}_collision">{pose}{geometry}{_surface(0.8)}</collision>'
                   f'<visual name="{name}_visual">{pose}{geometry}<material><ambient>{colour}</ambient>'
                   f"<diffuse>{colour}</diffuse></material></visual>")
    for name, radius, height, centre in cylinders(scene):
        geometry = f"<geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry>"
        pose = f"<pose>{centre[0]} {centre[1]} {centre[2]} 0 0 0</pose>"
        out.append(f'<collision name="{name}_collision">{pose}{geometry}</collision>'
                   f'<visual name="{name}_visual">{pose}{geometry}<material><ambient>0.8 0.2 0.2 1</ambient>'
                   "<diffuse>0.8 0.2 0.2 1</diffuse></material></visual>")
    out.append("</link></model>")
    dock = scene["dock"]
    out.append(f'<include><name>apriltag_dock</name><uri>{dock["model_uri"]}</uri>'
               f'<pose>{" ".join(str(v) for v in value(dock["pose"]))}</pose></include>')
    # The marker plate of openamr-platform-sw #167 is an opt-in extension (Alex, 18 Nov 2026): only with station_alignment.
    if station_marker and "station_marker" in scene:
        x, y, z, yaw = station_marker_pose(scene)
        out.append(f'<include><name>station_marker</name><uri>{scene["station_marker"]["model_uri"]}</uri>'
                   f"<pose>{x:.5f} {y:.5f} {z:.5f} 0 0 {yaw:.5f}</pose></include>")
    for name, body in movable_objects(scene).items():
        out.append(_movable_sdf(name, body))
    if extension(scene) is not None:
        out += extension(scene).world_elements(scene)     # visual-only products and lights of the extension
    out += ["</world>", "</sdf>", ""]
    return "\n".join(out)


# ---------------------------------------------------------------- Nav2 map

def _box_corners(size, centre, yaw):
    hx, hy = size[0] / 2.0, size[1] / 2.0
    return [(centre[0] + dx, centre[1] + dy) for dx, dy in (_rot(sx * hx, sy * hy, yaw) for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1)))]


def _inside(point, corners):
    px, py = point
    sign = 0
    for (ax, ay), (bx, by) in zip(corners, corners[1:] + corners[:1]):
        cross = (bx - ax) * (py - ay) - (by - ay) * (px - ax)
        if abs(cross) < 1e-12:
            continue
        if sign == 0:
            sign = 1 if cross > 0 else -1
        elif (cross > 0) != (sign > 0):
            return False
    return True


LIDAR_PLANE_Z = 0.222     # m above the floor: M-01, LiDAR scan plane 222.05 mm


def occupancy_grid(scene):
    """-> (rows of 0 free / 100 occupied, top row first, resolution, origin xy). Occupied cells are what the
    LiDAR plane at 0.222 m cuts: shelf posts, obstacles and the dock panel. The floor edge is a border."""
    resolution = value(scene["map"]["resolution"])
    fx, fy = value(scene["floor"]["size"])
    margin = scene["map"]["margin"]
    # The floor is centred on the world origin.
    min_x, min_y = -fx / 2.0 - margin, -fy / 2.0 - margin
    width = int(round((fx + 2 * margin) / resolution))
    height = int(round((fy + 2 * margin) / resolution))
    shapes = []
    for _name, size, centre, yaw in static_boxes(scene):
        if centre[2] - size[2] / 2.0 <= LIDAR_PLANE_Z <= centre[2] + size[2] / 2.0:
            shapes.append(_box_corners(size, centre, yaw))
    dock = value(scene["dock"]["pose"])
    if dock[2] - 0.10 <= LIDAR_PLANE_Z <= dock[2] + 0.10:    # the 0.20 m high backing of the dock model
        shapes.append(_box_corners((0.02, 1.10, 0.2), (dock[0], dock[1], dock[2]), dock[5]))
    round_shapes = [(c[0], c[1], r) for _n, r, h, c in cylinders(scene) if c[2] - h / 2.0 <= LIDAR_PLANE_Z <= c[2] + h / 2.0]
    grid = [[0] * width for _ in range(height)]
    half = resolution / 2.0
    for corners in shapes:
        xs, ys = [c[0] for c in corners], [c[1] for c in corners]
        for row in range(max(0, int((min(ys) - min_y) / resolution) - 1), min(height, int((max(ys) - min_y) / resolution) + 2)):
            for col in range(max(0, int((min(xs) - min_x) / resolution) - 1), min(width, int((max(xs) - min_x) / resolution) + 2)):
                cx, cy = min_x + col * resolution + half, min_y + row * resolution + half
                # A cell is occupied when its centre or one of its corners lies in the shape.
                if any(_inside(p, corners) for p in ((cx, cy), (cx - half, cy - half), (cx + half, cy - half),
                                                     (cx - half, cy + half), (cx + half, cy + half))):
                    grid[row][col] = 100
    for cx0, cy0, radius in round_shapes:
        for row in range(height):
            for col in range(width):
                cx, cy = min_x + col * resolution + half, min_y + row * resolution + half
                if math.hypot(cx - cx0, cy - cy0) <= radius + half:
                    grid[row][col] = 100
    return grid[::-1], resolution, (min_x, min_y)


def write_map(scene, directory, name="build1_map"):
    """Write <name>.pgm and <name>.yaml for nav2_map_server. -> path of the yaml."""
    grid, resolution, origin = occupancy_grid(scene)
    os.makedirs(directory, exist_ok=True)
    with open(os.path.join(directory, f"{name}.pgm"), "wb") as f:
        f.write(f"P5\n{len(grid[0])} {len(grid)}\n255\n".encode())
        for row in grid:
            f.write(bytes(0 if cell else 254 for cell in row))
    path = os.path.join(directory, f"{name}.yaml")
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump({"image": f"{name}.pgm", "mode": "trinary", "resolution": resolution,
                        "origin": [origin[0], origin[1], 0.0], "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.25}, f)
    return path


def collision_boxes(scene):
    """Every box MoveIt should know, in the world: the fixed boxes, the tote, the item and the payload.
    -> [(name, size, centre xyz, yaw)]. The tote is one box of its outer size."""
    boxes = list(static_boxes(scene))
    for name, body in movable_objects(scene).items():
        x, y, z, yaw = body["pose"]
        size = body["size"]
        centre_z = z + size[2] / 2.0 if body["kind"] == "tote" else z
        boxes.append((name, size, (x, y, centre_z), yaw))
    return boxes
