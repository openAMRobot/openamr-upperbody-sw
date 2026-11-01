#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Canary for G02's "no gz server of this run survives the teardown" (P16 task 2): proves the check FAILS on an orphaned simulator.
#
# It runs the real G02 launch test against a COPY of the installed package whose sim.launch.py has the stop handler (`stop_server`, the
# helper that signals the server) left out. Without it the launch's SIGINT reaches only the shell around `gz sim` and the server is left
# running: the documented way the simulator is orphaned. The copy is put first on AMENT_PREFIX_PATH; nothing in the checkout or the install
# is changed.
#
# Exit 0 when G02 FAILED because a `gz sim` of its own partition survived (the check works), 1 when it did not (the check is blind),
# 2 when the canary could not be built. A natural orphan is not needed and is not what this shows: it shows the check, through the whole
# launch-test path, on a server that really is left behind.
#
#   source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run ... or:  bash <this file>
set -u
GAZEBO=openamrobot_upperbody_gazebo
command -v colcon >/dev/null 2>&1 || { echo "canary_gz_orphan: colcon not on PATH: source ROS and the workspace first" >&2; exit 2; }
SHARE="$(python3 -c "from ament_index_python.packages import get_package_share_directory as g; print(g('$GAZEBO'))")" || exit 2
WS="${OPENAMR_WS:-$(cd "$SHARE/../../../.." && pwd)}"
[ -d "$WS/src" ] && [ -d "$WS/install" ] || { echo "canary_gz_orphan: $WS is not the workspace (no src/ and install/); set OPENAMR_WS" >&2; exit 2; }
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

mkdir -p "$WORK/prefix/share/ament_index/resource_index/packages" && touch "$WORK/prefix/share/ament_index/resource_index/packages/$GAZEBO"
cp -rL "$SHARE" "$WORK/prefix/share/$GAZEBO" && rm -rf "$WORK/prefix/share/$GAZEBO/launch/__pycache__"
python3 - "$WORK/prefix/share/$GAZEBO/launch/sim.launch.py" <<'PY' || exit 2
import sys
path = sys.argv[1]
text = open(path).read()
line = "        stop_server,\n"
if text.count(line) != 1:
    sys.exit("canary_gz_orphan: sim.launch.py no longer lists stop_server once in its LaunchDescription: the canary needs updating")
open(path, "w").write(text.replace(line, "        # CANARY: the stop handler is left out\n"))
PY

export AMENT_PREFIX_PATH="$WORK/prefix:$AMENT_PREFIX_PATH"
export OPENAMR_EVIDENCE_DIR="$WORK/evidence"; mkdir -p "$OPENAMR_EVIDENCE_DIR"
LOG="$WORK/g02.log"
( cd "$WS" && colcon test --packages-select "$GAZEBO" --ctest-args -R g02 --event-handlers console_direct+ ) > "$LOG" 2>&1

grep -q "Test *#[0-9]*: .*g02" "$LOG" || { echo "canary_gz_orphan: the G02 launch test did not run; log tail:" >&2; tail -n 20 "$LOG" >&2; exit 2; }
if grep -q "gz sim process(es) of this run" "$LOG" || grep -q "survived the launch's teardown" "$LOG" \
   || python3 -c "import json,sys; d=json.load(open('$OPENAMR_EVIDENCE_DIR/G02_gripper_mimic.json')); sys.exit(0 if d['result']=='FAIL' and d['details'].get('gz_server_after_shutdown',{}).get('survivors') else 1)" 2>/dev/null; then
  echo "canary_gz_orphan: PASS (the check caught the orphan): $(python3 -c "import json; d=json.load(open('$OPENAMR_EVIDENCE_DIR/G02_gripper_mimic.json'))['details']['gz_server_after_shutdown']; print('survivors', [(s['pid'], s['ppid']) for s in d['survivors']], 'reaped', d['reaped'])")"
  exit 0
fi
echo "canary_gz_orphan: FAIL (G02 did not report the server that was left running); log tail:" >&2
tail -n 30 "$LOG" >&2
exit 1
