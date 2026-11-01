#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""P25: the colour of the placeholder boxes in an RViz capture and a Gazebo capture, against the URDF's rgba (0.78 0.64 0.45).
Pixels of the 3D viewport whose hue is within 12 degrees of the material's and whose saturation is 0.15 to 0.60 (the boxes; the white base,
grey arms and grid are outside, the grid's and floor's grey is below 0.10) are taken; the median is reported. The two renderers light the box differently (RViz: ambient plus one
headlight; Gazebo: sun plus ambient), so brightness is expected to differ and the comparison that is meaningful is hue and the channel ratios.
usage: colour_compare.py out.json"""
import colorsys, json, sys
import numpy as np
from PIL import Image

URDF = np.array([0.78, 0.64, 0.45])
SHOTS = {
    "rviz_after": ("rviz/rviz_L1000_bp050.png", (715, 60, 1500, 915)),
    "gazebo_after": ("gazebo/gazebo_L1000_bp050.png", (0, 150, 580, 840)),
    "rviz_before_p19": ("../renders/rviz_L1000_bp050.png", None),
}
hue_ref = colorsys.rgb_to_hsv(*URDF)[0] * 360


def sample(path, box, hue_centre=hue_ref, tol=12.0):
    im = Image.open(path).convert("RGB")
    if box:
        im = im.crop(box)
    a = np.asarray(im, dtype=float) / 255.0
    flat = a.reshape(-1, 3)
    hsv = np.array([colorsys.rgb_to_hsv(*p) for p in flat[::1]])
    h = hsv[:, 0] * 360
    dh = np.minimum(abs(h - hue_centre), 360 - abs(h - hue_centre))
    keep = (dh < tol) & (hsv[:, 1] > 0.15) & (hsv[:, 1] < 0.60) & (hsv[:, 2] > 0.25)
    px = flat[keep]
    return px


out = {"urdf_rgba": URDF.tolist(), "urdf_hue_deg": hue_ref, "urdf_ratio_to_max": (URDF / URDF.max()).tolist()}
for name, (path, box) in SHOTS.items():
    if name == "rviz_before_p19":
        im = Image.open(path).convert("RGB"); a = np.asarray(im, dtype=float) / 255.0
        flat = a.reshape(-1, 3); red = flat[(flat[:, 0] > 0.6) & (flat[:, 1] < 0.2) & (flat[:, 2] < 0.2)]
        out[name] = {"pixels": int(len(red)), "median_rgb": (np.median(red, axis=0).tolist() if len(red) else None), "note": "red pixels of the P19 capture of the same configuration (platform-sw d619012)"}
        continue
    px = sample(path, box)
    med = np.median(px, axis=0)
    hsv = colorsys.rgb_to_hsv(*med)
    out[name] = {"pixels": int(len(px)), "median_rgb": med.tolist(), "hue_deg": hsv[0] * 360, "saturation": hsv[1], "value": hsv[2],
                 "ratio_to_max": (med / med.max()).tolist(),
                 "scale_vs_urdf_green_channel": float(med[1] / URDF[1])}
    if name == "gazebo_after":
        ratio = URDF / URDF.max()
        out[name]["urdf_ratio_to_max_to_the_1_over_2_2"] = (ratio ** (1 / 2.2)).tolist()
        out[name]["note"] = ("same hue, less saturated: the observed channel ratios are close to the URDF's raised to 1/2.2, which is what a renderer that takes "
                             "the rgba as linear light and sRGB-encodes the result would give. A reading of the numbers, not a verified fact about gz-rendering.")
json.dump(out, open(sys.argv[1], "w"), indent=1)
print(json.dumps(out, indent=1))
