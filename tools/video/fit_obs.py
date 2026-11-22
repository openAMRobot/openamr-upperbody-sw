# SPDX-License-Identifier: MIT
"""fit_obs.py B1_NPZ OBS_DIR: estimate the Gazebo time of observer frame 0 (1 Hz camera) by aligning frame-to-frame
image change with the recorded ground-truth base speed. Prints the offset; exits 1 when the fit is not distinct."""
import sys, glob
import numpy as np
from PIL import Image
z = np.load(sys.argv[1], allow_pickle=True); gt = z["build1__ground_truth__odom"]
fr = [np.asarray(Image.open(f).convert("L"), dtype=float) for f in sorted(glob.glob(sys.argv[2] + "/f*.png"))]
mot = np.array([np.abs(b - a).mean() for a, b in zip(fr, fr[1:])])          # change between frame k and k+1
t = gt[:, 0]; sp = np.hypot(np.gradient(gt[:, 1], t), np.gradient(gt[:, 2], t)) + 0.3 * np.abs(np.gradient(np.unwrap(gt[:, 3]), t))
drive = t <= t[sp > 0.02][-1] + 3.0                                           # fit on the drive, before arm motion dominates
k_max = int(min(len(mot), t[drive][-1] - t[0]))
best = []
for off in np.arange(t[0] - 3.0, t[0] + 6.0, 0.1):
    mid = off + np.arange(k_max) + 0.5
    s = np.interp(mid, t, sp, left=0, right=0); m = mot[:k_max]
    best.append((np.corrcoef(s, m)[0, 1] if s.std() > 0 and m.std() > 0 else -1, off))
best.sort(reverse=True); c, off = best[0]
print(f"fit: offset {off:.1f} s, correlation {c:.3f}, frames {len(fr)}, bag starts {t[0]:.2f} s", file=sys.stderr)
if not c > 0.5: sys.exit("observer alignment not distinct (correlation %.3f)" % c)
print(f"{off:.1f}")
