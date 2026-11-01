"""Fingertip gap of the pinch gripper vs finger_joint1 (finger_joint2 mimics it, multiplier 1), from the expanded URDF and the collision STLs.
Pure forward kinematics of two revolute joints; no simulator. Prints min vertex-to-vertex distance between the two finger collision meshes."""
import sys, struct, numpy as np
import xml.etree.ElementTree as ET
from scipy.spatial import cKDTree
urdf, side = sys.argv[1], sys.argv[2]
root = ET.parse(urdf).getroot()
J = {j.get('name'): j for j in root.findall('joint')}
L = {l.get('name'): l for l in root.findall('link')}
base = '/home/slash/openamr_ws/src/upstream/openarm_description/assets/end_effector/pinch_gripper/meshes/collision/'
def stl(path):
    d = open(path, 'rb').read()
    if d[:5] == b'solid' and b'facet' in d[:300]:
        import re
        v = np.array(re.findall(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)', d), dtype=float)
        return np.unique(v, axis=0)
    n = struct.unpack('<I', d[80:84])[0]
    a = np.frombuffer(d[84:84 + n * 50], dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
    return np.unique(a['v'].reshape(-1, 3).astype(float), axis=0)
def rot(axis, th):
    a = np.array(axis, float); a /= np.linalg.norm(a); K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K
def verts(link, joint, q):
    c = L[link].find('collision'); m = c.find('geometry/mesh'); sc = np.array(m.get('scale').split(), float)
    v = stl(base + m.get('filename').split('/')[-1]) * sc
    o = np.array(c.find('origin').get('xyz').split(), float); v = v + o
    jo = np.array(J[joint].find('origin').get('xyz').split(), float); ax = J[joint].find('axis').get('xyz').split()
    return (rot(ax, q) @ v.T).T + jo        # in the gripper base frame (the joint's parent), the base's own offset omitted: same for both fingers
j1, j2 = f'openarm_{side}_finger_joint1', f'openarm_{side}_finger_joint2'
lo, hi = [float(J[j1].find('limit').get(k)) for k in ('lower', 'upper')]
for label, q in (('q=0 (the shared end of both ranges)', 0.0), ('q=far end of the range', lo if abs(lo) > abs(hi) else hi), ('q=half', (lo if abs(lo) > abs(hi) else hi) / 2)):
    a = verts(f'openarm_{side}_ee_link1', j1, q); b = verts(f'openarm_{side}_ee_link2', j2, q)
    d, _ = cKDTree(b).query(a)
    print(f'{side:5s} {label:40s} q={q:+.4f}  min distance between the finger collision meshes {d.min()*1000:7.2f} mm   y-extent gap {(b[:,1].min()-a[:,1].max())*1000 if a[:,1].mean()<b[:,1].mean() else (a[:,1].min()-b[:,1].max())*1000:7.2f} mm')

print('--- tip separation: the 2 mm of each finger mesh farthest from its pivot (largest distance), their centroids, and the gap between those centroids')
def tips(v, pivot):
    r = np.linalg.norm(v - pivot, axis=1); return v[r > r.max() - 0.002]
for label, q in (('q=0', 0.0), ('q=0.25 of range', (lo if abs(lo) > abs(hi) else hi) * 0.25), ('q=half', (lo if abs(lo) > abs(hi) else hi) / 2), ('q=far end', lo if abs(lo) > abs(hi) else hi)):
    a = verts(f'openarm_{side}_ee_link1', j1, q); b = verts(f'openarm_{side}_ee_link2', j2, q)
    p1 = np.array(J[j1].find('origin').get('xyz').split(), float); p2 = np.array(J[j2].find('origin').get('xyz').split(), float)
    ta, tb = tips(a, p1), tips(b, p2)
    print(f'{side:5s} {label:18s} q={q:+.4f} tip1 centroid y={ta[:,1].mean()*1000:+7.2f} mm z={ta[:,2].mean()*1000:+8.2f} mm | tip2 y={tb[:,1].mean()*1000:+7.2f} mm z={tb[:,2].mean()*1000:+8.2f} mm | tip gap |dy|={abs(ta[:,1].mean()-tb[:,1].mean())*1000:6.2f} mm')
