#!/bin/sh
# SPDX-License-Identifier: MIT
set -u
R=$GITHUB_REPOSITORY
get() { mkdir -p "$2"; gh api "repos/$R/actions/artifacts/$1/zip" > "$2.zip" && unzip -q -o "$2.zip" -d "$2" -x '*/bag/*' '*.mp4' '*candump*' 2>/dev/null; }
get 14774058231 fail      # strict 45069865483 leg 1 (palm contact 1.35 N)
get 14774339918 pass      # strict 45069865483 leg 2
python3 -I tools/inspect/look.py
