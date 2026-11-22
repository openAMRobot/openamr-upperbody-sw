#!/bin/sh
# SPDX-License-Identifier: MIT
set -u
R=$GITHUB_REPOSITORY
get() { mkdir -p "$2"; gh api "repos/$R/actions/artifacts/$1/zip" > "$2.zip" && unzip -q -o "$2.zip" -d "$2"; }
get 14774253329 a132      # scene 45072808679 attempt 1, drive/lift/cancel (scn02 refusal, #132)
get 14775933244 a18       # scene 45075524313 attempt 1, software Stop faults on be4791c (scn18 FAIL)
get 14775645519 aclk      # strict 45075519060 leg 1 attempt 1 (no /clock)
echo "=== files a132"; find a132 -type f | sed 's/^/  /' | head -60
echo "=== files a18";  find a18 -type f | sed 's/^/  /' | head -60
echo "=== files aclk"; find aclk -type f -printf '  %p %s\n' | head -80
python3 -I tools/inspect/look.py
