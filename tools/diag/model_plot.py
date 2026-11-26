#!/usr/bin/env python3
# Diagnostic: link tree, zero-pose transforms and a two-view silhouette of the robot model from /robot_description.
import math, os, sys, xml.etree.ElementTree as ET
import numpy as np
urdf = open(sys.argv[1]).read(); out = sys.argv[2]
root = ET.fromstring(urdf)
def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])
def tf(el):
    T = np.eye(4)
    if el is not None:
        T[:3, 3] = [float(v) for v in el.get('xyz', '0 0 0').split()]
        T[:3, :3] = rpy(*[float(v) for v in el.get('rpy', '0 0 0').split()])
    return T
joints = {}
children = {}
for j in root.findall('joint'):
    p, c = j.find('parent').get('link'), j.find('child').get('link')
    lim = j.find('limit'); ax = j.find('axis')
    joints[c] = (j.get('name'), j.get('type'), p, tf(j.find('origin')), ax.get('xyz') if ax is not None else '',
                 (lim.get('lower', ''), lim.get('upper', '')) if lim is not None else ('', ''))
    children.setdefault(p, []).append(c)
links = [l.get('name') for l in root.findall('link')]
roots = [l for l in links if l not in joints]
world = {}
def walk(l, T):
    world[l] = T
    for c in children.get(l, []):
        walk(c, T @ joints[c][3])
for r in roots: walk(r, np.eye(4))
print('MODEL links', len(links), 'joints', len(joints), 'roots', roots)
print('JOINT\tname\ttype\tparent\tchild\taxis\tlower\tupper\tchild_x\tchild_y\tchild_z')
for c, (n, t, p, T, ax, lim) in joints.items():
    x, y, z = world[c][:3, 3]
    print(f'JOINT\t{n}\t{t}\t{p}\t{c}\t{ax}\t{lim[0]}\t{lim[1]}\t{x:.4f}\t{y:.4f}\t{z:.4f}')
pts = []; loaded = failed = 0
try:
    import trimesh
    from ament_index_python.packages import get_package_share_directory
    rng = np.random.default_rng(0)
    for l in root.findall('link'):
        for v in l.findall('visual'):
            g = v.find('geometry'); T = world[l.get('name')] @ tf(v.find('origin'))
            m = g.find('mesh') if g is not None else None
            try:
                if m is not None:
                    fn = m.get('filename'); sc = [float(s) for s in m.get('scale', '1 1 1').split()]
                    if fn.startswith('package://'):
                        pkg, rel = fn[10:].split('/', 1); fn = os.path.join(get_package_share_directory(pkg), rel)
                    elif fn.startswith('file://'): fn = fn[7:]
                    mesh = trimesh.load(fn, force='mesh'); V = np.asarray(mesh.vertices) * sc
                else:
                    b, c, s = g.find('box'), g.find('cylinder'), g.find('sphere')
                    if b is not None: mesh = trimesh.creation.box([float(x) for x in b.get('size').split()])
                    elif c is not None: mesh = trimesh.creation.cylinder(float(c.get('radius')), float(c.get('length')))
                    elif s is not None: mesh = trimesh.creation.icosphere(radius=float(s.get('radius')))
                    else: continue
                    V = np.asarray(mesh.sample(1500))
                if len(V) > 6000: V = V[rng.choice(len(V), 6000, replace=False)]
                pts.append((np.c_[V, np.ones(len(V))] @ T.T)[:, :3]); loaded += 1
            except Exception as e:
                failed += 1; print('MESHFAIL', l.get('name'), str(e)[:120])
except Exception as e:
    print('PLOT-IMPORT-FAIL', e)
print('VISUALS loaded', loaded, 'failed', failed)
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
P = np.vstack(pts) if pts else np.zeros((1, 3))
print('EXTENT min', P.min(0).round(3), 'max', P.max(0).round(3))
fig, axs = plt.subplots(1, 2, figsize=(11, 8), dpi=90)
for ax, (i, k, title, xl) in zip(axs, [(0, 2, 'Side view (x forward)', 'x [m]'), (1, 2, 'Front view (y left)', 'y [m]')]):
    ax.scatter(P[:, i], P[:, k], s=0.15, c='#8a8a8a', linewidths=0)
    for c, (n, t, p, T, a, lim) in joints.items():
        ax.plot([world[p][i, 3], world[c][i, 3]], [world[p][k, 3], world[c][k, 3]], c='#c0392b', lw=0.6)
    ax.set_aspect('equal'); ax.set_title(title); ax.set_xlabel(xl); ax.set_ylabel('z [m]'); ax.grid(alpha=0.3)
fig.suptitle('Robot model at zero joint positions: visual geometry (grey) and joint tree (red)')
fig.tight_layout(); fig.savefig(out)
print('PLOT saved', os.path.getsize(out))
