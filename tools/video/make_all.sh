#!/bin/sh
# SPDX-License-Identifier: MIT
# Regenerates video_b_grocery.mp4 and video_c_engineering.mp4 from the recorded artifacts. Run from an empty work
# directory; T is this tools/video directory. Needs: gh, python3 with rosbags matplotlib numpy pillow, ffmpeg.
set -e
T=$(cd "$(dirname "$0")" && pwd); R=openAMRobot/openamr-upperbody-sw
mkdir -p in_b1 in_gd obs gfr
gh run download 44309791416 -R $R -n baseline1-acceptance-attempt1 -D in_b1
gh run download 44299879412 -R $R -n grocery-demo-attempt1 -D in_gd
ffmpeg -v error -i in_b1/ws/build1_logs/baseline1_observer.mp4 obs/f%03d.png
ffmpeg -v error -i in_gd/ws/build1_logs/grocery_demo.mp4 gfr/f%03d.png
P="python3 -I"
$P $T/extract_b1.py in_b1/ws/build1_logs/bag b1.npz
$P $T/extract_b1x.py in_b1/ws/build1_logs/bag b1x.npz
$P $T/parse_can.py in_b1/ws/build1_logs in_b1/ws/build1_logs/baseline1_mission.json b1_can.npz
$P $T/extract_gd.py in_gd/ws/build1_logs/bag gd.npz
$P $T/extract_gd2.py in_gd/ws/build1_logs/bag gd2.npz
$P $T/parse_can.py in_gd/ws/build1_logs in_gd/ws/build1_logs/grocery_demo_mission.json gd_can.npz
$P $T/cards.py c; $P $T/cards.py b; $P $T/slides_c.py
$P $T/render_c.py part 0 2 & $P $T/render_c.py part 1 2 & wait
for s in 0 1 2 3 4 5; do $P $T/render_b2.py section $s; done
sh $T/assemble.sh video_c_engineering.mp4 card_c_intro.png 9 c_t1_components.png 10 c_part0.mp4 - c_part1.mp4 - \
  c_t2_can.png 14 c_t3_controllers.png 14 c_t4_physics.png 12 c_t5_clock.png 12 c_t6_limits.png 14 card_c_outro.png 8
sh $T/assemble.sh video_b_grocery.mp4 card_b_intro.png 7 b2_s0.mp4 - b2_s1.mp4 - b2_s2.mp4 - b2_s3.mp4 - b2_s4.mp4 - \
  b2_s5.mp4 - card_b_outro.png 5
