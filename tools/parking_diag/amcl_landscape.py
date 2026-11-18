# SPDX-License-Identifier: MIT
"""Offline: Nav2 AMCL likelihood-field score at the work-shelf station on the generated Build 1 map, for scans
ray-cast against the true scene geometry in the LiDAR plane. Shows where the scan alone puts the base."""
import math, sys
sys.path.insert(0, sys.argv[1])
import scene_mod as S
sc = S.load(sys.argv[1] + "/scene.yaml")
grid, res, (mx, my) = S.occupancy_grid(sc)
g = grid[::-1]; H, W = len(g), len(g[0])
occ = [(r, c) for r in range(H) for c in range(W) if g[r][c] == 100]
MAXD = 2.0
# distance map (cell units) brute force on a window around the station only
def dist_cell(r, c):
    best = MAXD
    for (ro, co) in occ:
        d = math.hypot(ro - r, co - c) * res
        if d < best: best = d
    return best
cache = {}
def z_at(x, y):
    c = int(math.floor((x - mx) / res)); r = int(math.floor((y - my) / res))
    if not (0 <= r < H and 0 <= c < W): return MAXD
    if (r, c) not in cache: cache[(r, c)] = dist_cell(r, c)
    return cache[(r, c)]
# true geometry: polygons + circles in the LiDAR plane (same objects the map is made from)
polys = [S._box_corners(size, centre, yaw) for _n, size, centre, yaw in S.static_boxes(sc)
         if centre[2] - size[2] / 2 <= S.LIDAR_PLANE_Z <= centre[2] + size[2] / 2]
dock = S.value(sc["dock"]["pose"])
polys.append(S._box_corners((0.02, 1.10, 0.2), (dock[0], dock[1], dock[2]), dock[5]))
circles = [(c[0], c[1], r) for _n, r, h, c in S.cylinders(sc)]
def ray(ox, oy, a, rmin=0.40, rmax=10.0):
    dx, dy = math.cos(a), math.sin(a); best = math.inf
    for P in polys:
        for i in range(len(P)):
            (x1, y1), (x2, y2) = P[i][:2], P[(i + 1) % len(P)][:2]
            ex, ey = x2 - x1, y2 - y1; den = dx * ey - dy * ex
            if abs(den) < 1e-12: continue
            t = ((x1 - ox) * ey - (y1 - oy) * ex) / den; u = ((x1 - ox) * dy - (y1 - oy) * dx) / den
            if t > 0 and 0 <= u <= 1: best = min(best, t)
    for cx, cy, rr in circles:
        fx, fy = ox - cx, oy - cy; b = fx * dx + fy * dy; c = fx * fx + fy * fy - rr * rr; disc = b * b - c
        if disc >= 0:
            t = -b - math.sqrt(disc)
            if t > 0: best = min(best, t)
    return best if rmin <= best <= rmax else math.inf
LIDAR_X, LIDAR_YAW = 0.330, math.pi
MASK = (math.radians(-35.0), math.radians(31.0))
def scan(pose):
    x, y, th = pose
    lx, ly = x + LIDAR_X * math.cos(th), y + LIDAR_X * math.sin(th)
    out = []
    for i in range(360):
        a = -math.pi + i * (2 * math.pi / 359)
        if MASK[0] <= a <= MASK[1]: out.append(math.inf); continue
        out.append(ray(lx, ly, th + LIDAR_YAW + a))
    return out
def score(ranges, pose, z_hit=0.5, z_rand=0.5, sigma=0.2, max_beams=60, rmax=12.0):
    x, y, th = pose; lx, ly = x + LIDAR_X * math.cos(th), y + LIDAR_X * math.sin(th)
    step = (len(ranges) - 1) // (max_beams - 1); p = 1.0; used = 0
    for i in range(0, len(ranges), step):
        r = ranges[i]
        if not math.isfinite(r) or r >= 10.0: continue
        a = th + LIDAR_YAW + (-math.pi + i * (2 * math.pi / 359))
        z = z_at(lx + r * math.cos(a), ly + r * math.sin(a))
        pz = z_hit * math.exp(-z * z / (2 * sigma * sigma)) + z_rand / rmax
        p += pz ** 3; used += 1
    return p, used
station = S.station(sc, "work_shelf")
truth = (station[0], station[1], station[2])
rs = scan(truth)
print("beams with a return", sum(1 for r in rs if math.isfinite(r)), "of 360")
base, used = score(rs, truth)
print("beams used by AMCL (max_beams 60)", used)
for sigma in (0.2, 0.05):
    best = None; grid_scores = {}
    for i in range(-16, 17):
        for j in range(-16, 17):
            for k in (-0.02, -0.01, 0.0, 0.01, 0.02):
                # offset expressed in the station frame: along (+ towards the shelf), left
                along, left = i * 0.005, j * 0.005
                g_ = station[2]
                px = truth[0] + along * math.cos(g_) - left * math.sin(g_)
                py = truth[1] + along * math.sin(g_) + left * math.cos(g_)
                s, _ = score(rs, (px, py, truth[2] + k), sigma=sigma)
                grid_scores[(i, j, k)] = s
                if best is None or s > best[0]: best = (s, along, left, k)
    s0 = grid_scores[(0, 0, 0.0)]
    print(f"sigma_hit {sigma}: score at truth {s0:.4f}, best {best[0]:.4f} at along {best[1]:+.3f} left {best[2]:+.3f} yaw {best[3]:+.2f}")
    for d in (0.02, 0.04, 0.06):
        n = int(round(d / 0.005))
        ring = [grid_scores[(i, j, 0.0)] for i in range(-n, n + 1) for j in range(-n, n + 1) if max(abs(i), abs(j)) == n]
        print(f"   score drop from truth at {d:.2f} m (square ring): min {s0 - max(ring):+.5f} max {s0 - min(ring):+.5f} (relative {100*(s0-min(ring))/s0:.2f} %)")
print("plateau (score within 0.5 % of the best) at yaw 0, sigma_hit 0.2, offsets of the pose from the truth:")
pts = []
for i in range(-16, 17):
    for j in range(-16, 17):
        along, left = i * 0.005, j * 0.005
        g_ = station[2]
        px = truth[0] + along * math.cos(g_) - left * math.sin(g_)
        py = truth[1] + along * math.sin(g_) + left * math.cos(g_)
        pts.append((score(rs, (px, py, truth[2]))[0], along, left))
top = max(p[0] for p in pts)
pl = [p for p in pts if p[0] >= top * 0.995]
print(f"  {len(pl)} of {len(pts)} grid poses; along {min(p[1] for p in pl):+.3f} to {max(p[1] for p in pl):+.3f}, left {min(p[2] for p in pl):+.3f} to {max(p[2] for p in pl):+.3f}")
pl2 = [p for p in pts if p[0] >= top * 0.98]
print(f"  within 2 %: {len(pl2)} poses; along {min(p[1] for p in pl2):+.3f} to {max(p[1] for p in pl2):+.3f}, left {min(p[2] for p in pl2):+.3f} to {max(p[2] for p in pl2):+.3f}")
# the same at the end of stage 1 (0.35 m before the station), where the final approach starts
s1 = (truth[0], truth[1] + 0.35, truth[2])
rs1 = scan(s1)
print("stage-1 end pose: returns", sum(1 for r in rs1 if math.isfinite(r)), "used", score(rs1, s1)[1])
