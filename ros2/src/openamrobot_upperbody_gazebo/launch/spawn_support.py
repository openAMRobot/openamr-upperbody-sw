# SPDX-License-Identifier: MIT
"""Place the zero-pose robot on its drive wheels and loaded castor pair.

The current CAD castors and approved drive axle are not coplanar. A level spawn
tips onto the front castors, imparting an impulse to arms starting at a limit.
Derive world placement from the consumed URDF, without changing any joint pose,
hardware dimension, mass, controller or limit. This is not six-contact fidelity.
"""
import math
import xml.etree.ElementTree as ET

# Numerical free space above the support plane, not a hardware dimension.
SPAWN_CLEARANCE_M = 1e-6
SUPPORT_LINKS = ('left_wheel_link', 'right_wheel_link', 'fl_wheel_link',
                 'fr_wheel_link', 'bl_wheel_link', 'br_wheel_link')


def _origin(point, element):
    origin = element.find('origin')
    if origin is None:
        return tuple(point)
    x, y, z = point
    roll, pitch, yaw = map(float, origin.get('rpy', '0 0 0').split())
    y, z = math.cos(roll)*y - math.sin(roll)*z, math.sin(roll)*y + math.cos(roll)*z
    x, z = math.cos(pitch)*x + math.sin(pitch)*z, -math.sin(pitch)*x + math.cos(pitch)*z
    x, y = math.cos(yaw)*x - math.sin(yaw)*y, math.sin(yaw)*x + math.cos(yaw)*y
    return tuple(a+b for a, b in zip((x, y, z), map(float, origin.get('xyz', '0 0 0').split())))


def ground_support_pose(urdf):
    """Return (world z, pitch) for the nominal zero-state support polygon.

Only the existing cylinder-drive/sphere-castor arrangement is supported. Fail
explicitly if a future geometry has no nonpenetrating, statically loaded support
plane; do not silently invent a new placement or change the contact geometry.
"""
    robot = ET.fromstring(urdf)
    parents = {j.find('child').get('link'): j for j in robot.findall('joint')}

    def point_in_root(link, element, point=(0., 0., 0.)):
        point = _origin(point, element)
        seen = set()
        while link in parents:
            if link in seen:
                raise ValueError('Cycle in the robot joint tree')
            seen.add(link)
            joint = parents[link]
            point = _origin(point, joint)  # all initial joint positions remain zero
            link = joint.find('parent').get('link')
        return point

    contacts = {}
    for name in SUPPORT_LINKS:
        link = robot.find(f"link[@name='{name}']")
        collisions = [] if link is None else link.findall('collision')
        if len(collisions) != 1:
            raise ValueError(f'Expected one support collision on {name}')
        collision = collisions[0]
        shape = collision.find('geometry/cylinder' if name in SUPPORT_LINKS[:2] else 'geometry/sphere')
        if shape is None:
            raise ValueError(f'Unsupported ground contact shape on {name}')
        center = point_in_root(name, collision)
        if name in SUPPORT_LINKS[:2]:
            endpoint = point_in_root(name, collision, (0., 0., 1.))
            axis = tuple(a-b for a, b in zip(endpoint, center))
            if abs(axis[0]) > 1e-7 or abs(axis[2]) > 1e-7:
                raise ValueError('Drive cylinder axes must be parallel to root Y')
        contacts[name] = (center, float(shape.get('radius')))

    mass = 0.
    weighted = [0., 0., 0.]
    for link in robot.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        m = float(inertial.find('mass').get('value'))
        center = point_in_root(link.get('name'), inertial)
        mass += m
        weighted = [v + m*c for v, c in zip(weighted, center)]
    if mass <= 0:
        raise ValueError('A positive nominal mass is required for support selection')
    com = [v / mass for v in weighted]

    def height(name, pitch):
        (x, _, z), radius = contacts[name]
        return radius + x*math.sin(pitch) - z*math.cos(pitch)

    # Test both possible loaded castor pairs. The nominal COM selects the
    # statically supported side; the unloaded pair must not penetrate the floor.
    for pair in [('fl_wheel_link', 'fr_wheel_link'), ('bl_wheel_link', 'br_wheel_link')]:
        def difference(pitch):
            return height(pair[0], pitch) - height('left_wheel_link', pitch)
        lo, hi = -0.1, 0.1
        if difference(lo)*difference(hi) > 0:
            continue
        for _ in range(60):
            mid = (lo + hi) / 2
            if difference(lo)*difference(mid) <= 0:
                hi = mid
            else:
                lo = mid
        pitch = (lo + hi) / 2
        z = height('left_wheel_link', pitch)
        loaded = SUPPORT_LINKS[:2] + pair
        if any(abs(height(n, pitch)-z) > 1e-7 for n in loaded):
            continue
        if any(height(n, pitch) > z + 1e-7 for n in SUPPORT_LINKS):
            continue
        # Project COM vertically into the trapezoidal four-contact polygon.
        def world_x(point):
            return point[0]*math.cos(pitch) + point[2]*math.sin(pitch)
        wheel_x = world_x(contacts['left_wheel_link'][0])
        caster_x = world_x(contacts[pair[0]][0])
        ratio = (world_x(com) - wheel_x) / (caster_x - wheel_x)
        y_min = (1-ratio)*contacts['right_wheel_link'][0][1] + ratio*contacts[pair[1]][0][1]
        y_max = (1-ratio)*contacts['left_wheel_link'][0][1] + ratio*contacts[pair[0]][0][1]
        if 0 <= ratio <= 1 and y_min <= com[1] <= y_max:
            return z + SPAWN_CLEARANCE_M, pitch
    raise ValueError('No supported wheel/castor plane for the nominal zero-pose robot')
