#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Encodes the store camera frames of the grocery dark store demonstrator into a video. Visual record, not evidence.

    grocery_video.py LOG_DIR [--fps 10] [--keep-frames]

Reads LOG_DIR/frames/*.ppm (named by their Gazebo stamp, 2 Hz of Gazebo time) and LOG_DIR/grocery_demo_report.json
(phase start and end in Gazebo time) and writes, with ffmpeg:
  grocery_demo.mp4        the frames at --fps (5x Gazebo time at 10 fps), a label and the running phase burned in
  grocery_demo.srt        the phases as subtitles (phase, verdict), for players that show them
  still_<phase>.png       one frame from the middle of lift_object, place and return_to_parking
  grocery_demo_frames.csv frame index, video second, Gazebo second: the video's playback time is index / fps, so
                          it is Gazebo time only while no frame is missing; this file is the join key to the bag,
                          the report and the control room (one time base, Gazebo time)
The frames are removed after a successful encode unless --keep-frames. No ROS import.
"""
import argparse
import glob
import json
import os
import subprocess
import sys

LABEL = "Gazebo simulation, sim-oracle profile. Visual record, not evidence"


def srt_time(seconds):
    ms = int(round(seconds * 1000))
    return f"{ms // 3600000:02d}:{ms // 60000 % 60:02d}:{ms // 1000 % 60:02d},{ms % 1000:03d}"


def frame_stamps(directory):
    paths = sorted(glob.glob(os.path.join(directory, "*.ppm")))
    stamps = []
    for path in paths:
        try:
            stamps.append(float(os.path.basename(path)[:-4]))
        except ValueError:
            continue
    return paths, stamps


def subtitles(phases, stamps, fps):
    """SRT entries [(start video s, end video s, text)] for the phases that have frames."""
    out = []
    for phase in phases:
        a, b = phase.get("sim_start"), phase.get("sim_end")
        if a is None or b is None:
            continue
        inside = [i for i, s in enumerate(stamps) if a <= s <= b]
        if not inside:
            continue
        out.append((inside[0] / fps, (inside[-1] + 1) / fps, f"{phase['phase']}: {phase['verdict']}"))
    return out


def frame_timeline(stamps, fps, nominal_period=0.5):
    """(rows, summary): rows (index, video s, Gazebo s) and the largest gap between frames against the nominal
    period (2 Hz of Gazebo time). Where frames are missing the video runs ahead of Gazebo time by the lost frames."""
    rows = [(i, round(i / fps, 3), round(s, 3)) for i, s in enumerate(stamps)]
    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    missing = sum(max(0, round(g / nominal_period) - 1) for g in gaps)
    return rows, {"frames": len(stamps), "largest_gap_s": round(max(gaps), 3) if gaps else None,
                  "missing_frames": missing, "gazebo_span_s": round(stamps[-1] - stamps[0], 3) if stamps else None}


def encode(frames_dir, output, fps, srt):
    base = ["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(fps), "-pattern_type", "glob", "-i",
            os.path.join(frames_dir, "*.ppm")]
    text = LABEL.replace(":", r"\:").replace(",", r"\,")
    label = f"drawtext=text='{text}':x=12:y=12:fontsize=18:fontcolor=white:box=1:boxcolor=black@0.55:boxborderw=5"
    tail = ["-c:v", "libx264", "-preset", "veryfast", "-crf", "24", "-pix_fmt", "yuv420p", output]
    attempts = [[*base, "-vf", f"{label},subtitles={srt}:force_style='FontSize=20'", *tail],
                [*base, "-vf", label, *tail], [*base, *tail]]
    for command in attempts:
        if os.path.exists(srt) or "subtitles" not in " ".join(command):
            done = subprocess.run(command, capture_output=True, text=True, check=False)
            if done.returncode == 0:
                return " ".join(command[command.index("-vf") + 1:command.index("-vf") + 2]) if "-vf" in command else "none"
            print(f"ffmpeg failed ({done.stderr.strip()[-300:]}), trying with fewer filters", flush=True)
    return None


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("log_dir")
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--keep-frames", action="store_true")
    args = parser.parse_args(argv)
    frames_dir = os.path.join(args.log_dir, "frames")
    paths, stamps = frame_stamps(frames_dir)
    print(f"store camera frames: {len(paths)}"
          + (f", Gazebo {stamps[0]:.1f} to {stamps[-1]:.1f} s" if stamps else ""), flush=True)
    if len(paths) < 2:
        print("fewer than two frames: no video", flush=True)
        return 1
    report_path = os.path.join(args.log_dir, "grocery_demo_report.json")
    phases = json.load(open(report_path, encoding="utf-8")).get("phases", []) if os.path.exists(report_path) else []
    srt = os.path.join(args.log_dir, "grocery_demo.srt")
    entries = subtitles(phases, stamps, args.fps)
    with open(srt, "w", encoding="utf-8") as f:
        for n, (a, b, text) in enumerate(entries, 1):
            f.write(f"{n}\n{srt_time(a)} --> {srt_time(b)}\n{text}\n\n")
    rows, timeline = frame_timeline(stamps, args.fps)
    with open(os.path.join(args.log_dir, "grocery_demo_frames.csv"), "w", encoding="utf-8") as f:
        f.write("frame,video_s,gazebo_s\n" + "".join(f"{i},{v},{g}\n" for i, v, g in rows))
    print(f"frame timeline: {json.dumps(timeline, sort_keys=True)} (grocery_demo_frames.csv)", flush=True)
    output = os.path.join(args.log_dir, "grocery_demo.mp4")
    used = encode(frames_dir, output, args.fps, srt)
    if used is None:
        print("ffmpeg could not encode the frames", flush=True)
        return 1
    print(f"video {output}: {len(paths)} frames at {args.fps} fps, filters {used}", flush=True)
    for name in ("lift_object", "place", "return_to_parking"):
        phase = next((p for p in phases if p["phase"] == name and p.get("sim_start") is not None and p.get("sim_end") is not None), None)
        if phase is None:
            continue
        middle = (phase["sim_start"] + phase["sim_end"]) / 2.0
        index = min(range(len(stamps)), key=lambda i: abs(stamps[i] - middle))
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", paths[index], os.path.join(args.log_dir, f"still_{name}.png")],
                       check=False)
    if not args.keep_frames:
        for path in paths:
            os.remove(path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
