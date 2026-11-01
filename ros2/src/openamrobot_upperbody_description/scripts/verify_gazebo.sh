#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Gazebo verification. Runs the launch tests of openamrobot_upperbody_gazebo (G01, G02: a headless Gazebo server each), then
# writes <workspace>/evidence/<UTC timestamp>_gazebo.json (openamr.evidence.v1, evidence_class simulation).
# Exit status: 1 if any check is FAIL, 3 if every check passed but the tree is dirty (the results print, and no evidence file is
# written: a report describes a commit and a dirty tree is none), 2 if the evidence cannot be assembled, else 0. V01 stays WARN:
# it needs a person (docs/v01_visual_check.md).
#
#   source /opt/ros/jazzy/setup.bash && source install/setup.bash
#   ros2 run openamrobot_upperbody_description verify_gazebo.sh        # or run this file directly
#
# Run `colcon build` first: the tests and this report describe the INSTALLED packages (I01 fails if they are stale).
# It takes about a minute and a half. Each test picks its own ROS_DOMAIN_ID and Gazebo partition, so a simulator you have
# open is neither seen nor stopped by it. OPENAMR_EVIDENCE_DIR overrides where evidence goes; OPENAMR_WS the workspace root.
set -u

PKG=openamrobot_upperbody_description
GAZEBO=openamrobot_upperbody_gazebo
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

workspace_root() {
  if [ -n "${OPENAMR_WS:-}" ]; then echo "$OPENAMR_WS"; return; fi
  local d="$HERE"
  while [ "$d" != "/" ]; do
    if [ -d "$d/install" ] && [ -d "$d/src" ]; then echo "$d"; return; fi
    d="$(dirname "$d")"
  done
  return 1
}

WS="$(workspace_root)" || { echo "verify_gazebo: cannot find the workspace (no ancestor with install/ and src/); set OPENAMR_WS" >&2; exit 2; }
for tool in colcon gz ros2 python3; do
  command -v "$tool" >/dev/null 2>&1 || { echo "verify_gazebo: '$tool' not on PATH: source /opt/ros/jazzy/setup.bash and $WS/install/setup.bash first" >&2; exit 2; }
done
python3 -c "import ament_index_python.packages as p; p.get_package_share_directory('$GAZEBO')" 2>/dev/null \
  || { echo "verify_gazebo: package $GAZEBO is not on the ament path: source $WS/install/setup.bash" >&2; exit 2; }

export PYTHONDONTWRITEBYTECODE=1        # compiled caches would make the checkout look dirty
START="$(date +%s.%N)"

echo "== verify_gazebo: workspace $WS"
EVIDENCE_DIR="$(python3 "$HERE/build_evidence.py" --profile gazebo --clear)" || exit 2
echo "== evidence directory: $EVIDENCE_DIR (previous Gazebo sidecars cleared)"
rm -f "$WS/build/$GAZEBO/test_results/$GAZEBO/"*.xunit.xml 2>/dev/null

echo; echo "== colcon test ($GAZEBO: G01, G02)"
( cd "$WS" && colcon test --packages-select "$GAZEBO" --event-handlers console_cohesion+ )
RC_COLCON=$?
( cd "$WS" && colcon test-result --test-result-base "build/$GAZEBO" --verbose 2>&1 | tail -n 25 )
# `colcon test` exits 0 whatever its tests did (demonstrated in P12A); `colcon test-result` exits 1 for a failing or erroring test.
( cd "$WS" && colcon test-result --test-result-base "build/$GAZEBO" >/dev/null 2>&1 ) || RC_COLCON=1

echo; echo "== evidence report"
python3 "$HERE/build_evidence.py" --profile gazebo --since "$START" --step "colcon_test=$RC_COLCON"
exit $?
