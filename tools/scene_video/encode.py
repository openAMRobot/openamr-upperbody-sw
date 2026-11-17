#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Turns the recorded frames of the scene video into scene.mp4, scene.gif, arms_closeup.mp4, three stills, a contact
sheet and capture.json. Visual capture, not a test result. What did not move is stated in the overlay and in
capture.json, from the numbers the task recorded.

    encode.py OUT_DIR WORKFLOW_SHA PINS_FILE
"""
import base64
import csv
import json
import os
import subprocess
import sys

import cv2
import numpy as np

FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"


def frames(directory):
    path = os.path.join(directory, "frames.csv")
    if not os.path.exists(path):
        return []
    return [(int(r["index"]), float(r["stamp_s"])) for r in csv.DictReader(open(path, encoding="utf-8"))]


def clip(directory, stamps, first, last, name):
    keep = [(i, s) for i, s in stamps if first <= s <= last]
    target = os.path.join(directory, name)
    os.makedirs(target, exist_ok=True)
    for n, (i, _s) in enumerate(keep):
        os.link(os.path.join(directory, f"{i:06d}.jpg"), os.path.join(target, f"{n:06d}.jpg"))
    return keep, target


def drawtext(lines, size):
    out = []
    for n, line in enumerate(lines):
        text = line.replace(":", r"\:").replace("'", "")
        out.append(f"drawtext=fontfile={FONT}:text='{text}':x=16:y={14 + n * int(size * 1.4)}:fontsize={size}"
                   ":fontcolor=white:box=1:boxcolor=black@0.55:boxborderw=6")
    return ",".join(out)


def encode(source, output, lines, size):
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", "15", "-i", os.path.join(source, "%06d.jpg"),
                    "-vf", drawtext(lines, size), "-c:v", "libx264", "-preset", "veryfast", "-pix_fmt", "yuv420p",
                    "-crf", "26", output], check=True)


def defects(task):
    """What did not move, from the task's own numbers. Thresholds: 0.01 m for the lift, 0.05 rad for a finger."""
    steps = {s["name"]: s for s in task.get("steps", [])}
    end = steps.get("end", {})
    lift = end.get("lift_m")
    lift_reported = [s.get("succeeded") for n, s in steps.items() if n.startswith("lift_to_stop_2")]
    fingers = {n: s.get("finger_joint1_rad") for n, s in steps.items() if n.endswith(("_gripper_open", "_gripper_close"))}
    result = {}
    if lift is None or abs(lift) < 0.01:
        result["lift"] = {"commanded_m": 0.35, "measured_end_m": lift, "action_reported_success": lift_reported,
                          "text": "lift_controller reported success, lift_joint stayed at 0.000 m in DART"}
    values = [v for v in fingers.values() if isinstance(v, (int, float))]
    if not values or max(abs(v) for v in values) < 0.05:
        result["grippers"] = {"commanded_rad": {"open": 0.7854, "closed": 0.0}, "measured_finger_joint1_rad": fingers,
                              "text": "both grippers stayed at 0 rad in DART (README Known limits)"}
    return result


def main(out, sha, pins_file):
    task_path = os.path.join(out, "task.json")
    task = json.load(open(task_path, encoding="utf-8")) if os.path.exists(task_path) else {}
    markers = task.get("markers", {})
    main_dir, arms_dir = os.path.join(out, "frames"), os.path.join(out, "arms_frames")
    stamps, arm_stamps = frames(main_dir), frames(arms_dir)
    found = defects(task)
    lines = ["visual capture, not a test result"]
    if found:
        lines.append("lift and grippers do not move in DART yet (known defects)" if len(found) == 2
                     else f"{' and '.join(found)} do not move in DART yet (known defect)")

    first = markers.get("start", stamps[0][1] if stamps else 0.0) - 2.0
    last = markers.get("end", stamps[-1][1] if stamps else 0.0) + 1.0
    keep, main_clip = clip(main_dir, stamps, first, last, "clip")
    encode(main_clip, os.path.join(out, "scene.mp4"), lines, 26)
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", os.path.join(out, "scene.mp4"), "-vf",
                    "fps=8,scale=640:-1:flags=lanczos,split[a][b];[a]palettegen[p];[b][p]paletteuse",
                    os.path.join(out, "scene.gif")], check=True)
    arms_keep = []
    if arm_stamps and "reach_begin" in markers:
        arms_keep, arms_clip = clip(arms_dir, arm_stamps, markers["reach_begin"] - 1.0, markers.get("reach_end", last) + 1.0, "arms_clip")
        if arms_keep:
            encode(arms_clip, os.path.join(out, "arms_closeup.mp4"), lines, 20)

    def nearest(t, table):
        return min(table, key=lambda e: abs(e[1] - t))

    stills = {}
    for name, table, directory in (("start", stamps, main_dir), ("reach", stamps, main_dir), ("end", stamps, main_dir),
                                   ("reach_closeup", arm_stamps, arms_dir)):
        key = "reach" if name == "reach_closeup" else name
        if key in markers and table:
            i, s = nearest(markers[key], table)
            path = os.path.join(out, f"still_{name}.png")
            cv2.imwrite(path, cv2.imread(os.path.join(directory, f"{i:06d}.jpg")))
            stills[name] = {"file": os.path.basename(path), "sim_s": s}

    steps = task.get("steps", [])
    end = next((s for s in steps if s["name"] == "end"), {})
    pins = [p for p in open(pins_file, encoding="utf-8").read().split("\n") if p] if os.path.exists(pins_file) else []
    capture = {
        "label": "visual capture, not a test result",
        "known_defects_seen_in_this_run": found,
        "workflow_sha": sha,
        "pins": pins,
        "profile": os.environ.get("PROFILE"), "physics_engine": os.environ.get("ENGINE"),
        "run_info": next((s.get("run_info") for s in steps if s["name"] == "ready"), None),
        "gazebo_rtf": end.get("rtf_stats"),
        "cameras": {"main": {"sdf": "tools/scene_video/observer_camera.sdf", "resolution": [1280, 720], "rate_hz_sim": 15},
                    "arms": {"sdf": "tools/scene_video/arms_camera.sdf", "resolution": [960, 540], "rate_hz_sim": 15}},
        "frames_recorded": len(stamps), "frames_in_video": len(keep), "arms_frames_recorded": len(arm_stamps),
        "arms_frames_in_closeup": len(arms_keep),
        "video_sim_seconds": round(keep[-1][1] - keep[0][1], 2) if len(keep) > 1 else 0.0,
        "video_fps": 15, "video_plays_at": "1x Gazebo time when no frame was dropped",
        "markers_sim_s": markers, "stills": stills, "commands": steps,
    }
    json.dump(capture, open(os.path.join(out, "capture.json"), "w", encoding="utf-8"), indent=2, default=str)
    print(json.dumps({k: capture[k] for k in ("frames_recorded", "frames_in_video", "arms_frames_in_closeup",
                                               "video_sim_seconds", "gazebo_rtf", "known_defects_seen_in_this_run")}, indent=2, default=str))

    # A contact sheet of six frames, printed into the log as base64: a reader without artifact access can check it.
    if keep:
        picks = [keep[int(k * (len(keep) - 1) / 5)] for k in range(6)]
        tiles = [cv2.resize(cv2.imread(os.path.join(main_dir, f"{i:06d}.jpg")), (640, 360)) for i, _s in picks]
        for tile, (_i, s) in zip(tiles, picks):
            cv2.putText(tile, f"t={s:.1f} s sim", (10, 350), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        sheet = np.vstack([np.hstack(tiles[0:2]), np.hstack(tiles[2:4]), np.hstack(tiles[4:6])])
        cv2.imwrite(os.path.join(out, "contact_sheet.jpg"), sheet, [cv2.IMWRITE_JPEG_QUALITY, 80])
        data = base64.b64encode(open(os.path.join(out, "contact_sheet.jpg"), "rb").read()).decode()
        for n in range(0, len(data), 4000):
            print("SHEET_B64 " + data[n:n + 4000])
    return 0


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:4]))
