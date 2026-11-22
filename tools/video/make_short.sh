#!/bin/sh
# SPDX-License-Identifier: MIT
# make_short.sh RUN_ID ARTIFACT "REF TEXT" "CHAIN TEXT" [SPEED]
# Renders one short engineering video from one recorded Baseline 1 acceptance artifact. Run from an empty work
# directory. Needs: gh (GH_TOKEN with actions:read), python3 with rosbags matplotlib numpy pillow, ffmpeg.
set -eu
T=$(cd "$(dirname "$0")" && pwd); R=${GITHUB_REPOSITORY:-openAMRobot/openamr-upperbody-sw}
RUN=$1; ART=$2; REF=$3; CHAIN=$4; SPEED=${5:-2}
mkdir -p in_b1 obs
gh run download "$RUN" -R "$R" -n "$ART" -D in_b1
MJ=$(find in_b1 -name baseline1_mission.json | head -1); [ -n "$MJ" ] || { echo "no baseline1_mission.json in $ART"; exit 1; }
LOG=$(dirname "$MJ"); ls -la "$LOG"
ffmpeg -v error -i "$LOG/baseline1_observer.mp4" obs/f%03d.png
P="python3 -I"
$P "$T/extract_b1.py" "$LOG/bag" b1.npz
$P "$T/parse_can.py" "$LOG" "$MJ" b1_can.npz
OBS_T0=$($P "$T/fit_obs.py" b1.npz obs)
export VIDEO_RUN="$RUN" VIDEO_ARTIFACT="$ART" VIDEO_REF="$REF" VIDEO_CHAIN="$CHAIN" VIDEO_SPEED="$SPEED" VIDEO_LOG="$LOG" \
  VIDEO_OBS_T0="$OBS_T0" VIDEO_AUTO_WINDOW=1
$P "$T/cards.py" short
$P "$T/render_c.py" part 0 4 & $P "$T/render_c.py" part 1 4 & $P "$T/render_c.py" part 2 4 & $P "$T/render_c.py" part 3 4 & wait
sh "$T/assemble.sh" openamrobot_digital_twin_run_$RUN.mp4 card_s_intro.png 4 c_part0.mp4 - c_part1.mp4 - c_part2.mp4 - \
  c_part3.mp4 - card_s_outro.png 5
# thumbnail: a frame from the middle of the mission, 1200x627 (LinkedIn link-preview size), letterboxed, not cropped
D=$(ffprobe -v error -show_entries format=duration -of csv=p=0 openamrobot_digital_twin_run_$RUN.mp4)
ffmpeg -v error -y -ss "$(python3 -I -c "print(float('$D')*0.62)")" -i openamrobot_digital_twin_run_$RUN.mp4 -frames:v 1 \
  -vf "scale=1200:627:force_original_aspect_ratio=decrease,pad=1200:627:(ow-iw)/2:(oh-ih)/2:color=0x1a1a19" thumbnail_run_$RUN.png
ffmpeg -v error -y -ss "$(python3 -I -c "print(float('$D')*0.62)")" -i openamrobot_digital_twin_run_$RUN.mp4 -frames:v 1 frame_1080p_run_$RUN.png
{ echo "source_run=$RUN"; echo "source_artifact=$ART"; echo "source_ref=$REF"; echo "speed=$SPEED"; echo "observer_frame0_gazebo_s=$OBS_T0"
  echo "duration_s=$D"; echo "renderer_sha=${GITHUB_SHA:-local}"; sha256sum openamrobot_digital_twin_run_$RUN.mp4 thumbnail_run_$RUN.png; } > provenance_run_$RUN.txt
cat provenance_run_$RUN.txt
