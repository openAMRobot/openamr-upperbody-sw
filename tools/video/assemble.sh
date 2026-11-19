#!/bin/sh
# SPDX-License-Identifier: MIT
# assemble.sh OUT  then pairs of (file seconds|"-") : PNG with a duration, or an MP4 with "-"
set -e
out=$1; shift; args=""; n=0; f=""
while [ $# -gt 0 ]; do
  src=$1; dur=$2; shift 2
  if [ "$dur" = "-" ]; then args="$args -i $src"; else args="$args -loop 1 -framerate 30 -t $dur -i $src"; fi
  f="$f[$n:v]fps=30,scale=1920:1080,setsar=1,format=yuv420p[v$n];"; n=$((n+1))
done
c=""; i=0; while [ $i -lt $n ]; do c="$c[v$i]"; i=$((i+1)); done
ffmpeg -v error -y $args -filter_complex "${f}${c}concat=n=$n:v=1:a=0[o]" -map "[o]" -c:v libx264 -preset medium -crf 20 \
  -profile:v high -pix_fmt yuv420p -r 30 -movflags +faststart -an "$out"
