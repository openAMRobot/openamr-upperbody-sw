# SPDX-License-Identifier: MIT
"""Offline what-if for the owners (no CI): flatness of the AMCL likelihood at the station for world/map variants."""
import math, sys, copy, runpy
sys.argv = [sys.argv[0], sys.argv[1]]
sys.path.insert(0, sys.argv[1])
import scene_mod as S
base_scene = S.load(sys.argv[1] + "/scene.yaml")
def run(scene, res=None, extra_boxes=()):
    sc = copy.deepcopy(scene)
    if res: sc["map"]["resolution"]["value"] = res
    orig_static = S.static_boxes
    S.static_boxes = lambda s: list(orig_static(s)) + list(extra_boxes)
    try:
        grid, r, (mx, my) = S.occupancy_grid(sc)
        g = grid[::-1]; H, W = len(g), len(g[0])
        occ = [(rr, c) for rr in range(H) for c in range(W) if g[rr][c] == 100]
        polys = [S._box_corners(size, centre, yaw) for _n, size, centre, yaw in S.static_boxes(sc)
                 if centre[2] - size[2] / 2 <= S.LIDAR_PLANE_Z <= centre[2] + size[2] / 2]
    finally:
        S.static_boxes = orig_static
    near = [(rr, c) for (rr, c) in occ if abs(mx + c * r - 0) < 4 and abs(my + rr * r + 2.4) < 4]
    cache = {}
    def z_at(x, y):
        c = int(math.floor((x - mx) / r)); rr = int(math.floor((y - my) / r))
        if (rr, c) not in cache:
            cache[(rr, c)] = min([math.hypot(ro - rr, co - c) * r for ro, co in near] + [2.0])
        return cache[(rr, c)]
    circles = [(c[0], c[1], rad) for _n, rad, h, c in S.cylinders(sc)]
    def ray(ox, oy, a):
        dx, dy = math.cos(a), math.sin(a); best = math.inf
        for P in polys:
            for i in range(len(P)):
                (x1, y1), (x2, y2) = P[i][:2], P[(i + 1) % len(P)][:2]
                ex, ey = x2 - x1, y2 - y1; den = dx * ey - dy * ex
                if abs(den) < 1e-12: continue
                t = ((x1 - ox) * ey - (y1 - oy) * ex) / den; u = ((x1 - ox) * dy - (y1 - oy) * dx) / den
                if t > 0 and 0 <= u <= 1: best = min(best, t)
        for cx, cy, rad in circles:
            fx, fy = ox - cx, oy - cy; b = fx * dx + fy * dy; cc = fx * fx + fy * fy - rad * rad; disc = b * b - cc
            if disc >= 0 and -b - math.sqrt(disc) > 0: best = min(best, -b - math.sqrt(disc))
        return best if 0.40 <= best <= 10.0 else math.inf
    st_ = S.station(sc, "work_shelf"); th = st_[2]
    lx, ly = st_[0] + 0.33 * math.cos(th), st_[1] + 0.33 * math.sin(th)
    angs = [-math.pi + i * (2 * math.pi / 359) for i in range(360)]
    rs = [math.inf if math.radians(-35) <= a <= math.radians(31) else ray(lx, ly, th + math.pi + a) for a in angs]
    def score(along, left):
        px = st_[0] + along * math.cos(th) - left * math.sin(th); py = st_[1] + along * math.sin(th) + left * math.cos(th)
        qx, qy = px + 0.33 * math.cos(th), py + 0.33 * math.sin(th); p = 1.0
        for i in range(0, 360, 6):
            if not math.isfinite(rs[i]): continue
            a = th + math.pi + angs[i]
            z = z_at(qx + rs[i] * math.cos(a), qy + rs[i] * math.sin(a))
            p += (0.5 * math.exp(-z * z / 0.08) + 0.5 / 12.0) ** 3
        return p
    pts = [(score(i * 0.005, j * 0.005), i * 0.005, j * 0.005) for i in range(-16, 17) for j in range(-16, 17)]
    top = max(p[0] for p in pts); pl = [p for p in pts if p[0] >= top * 0.98]
    used = sum(1 for i in range(0, 360, 6) if math.isfinite(rs[i]))
    return (sum(1 for x in rs if math.isfinite(x)), used, min(p[1] for p in pl), max(p[1] for p in pl), min(p[2] for p in pl), max(p[2] for p in pl), len(pl))
fmt = lambda name, v: print(f"{name:58s} returns {v[0]:3d} used {v[1]:2d} | within 2 %: along {v[2]:+.3f}..{v[3]:+.3f} left {v[4]:+.3f}..{v[5]:+.3f} ({v[6]} poses)")
fmt("as is (map 0.05 m)", run(base_scene))
fmt("map 0.025 m", run(base_scene, res=0.025))
x, y, yaw = base_scene["work_shelf"]["pose"]
plate = ("what_if_front_plate", (1.20, 0.02, 0.30), (x, y + 0.19, 0.15), yaw)
fmt("front plate 1.20 x 0.30 m at the shelf front (map 0.05)", run(base_scene, extra_boxes=[plate]))
sides = [("what_if_side_l", (0.02, 0.40, 0.30), (x - 0.59, y, 0.15), yaw), ("what_if_side_r", (0.02, 0.40, 0.30), (x + 0.59, y, 0.15), yaw)]
fmt("front plate and two side panels (map 0.05)", run(base_scene, extra_boxes=[plate] + sides))
