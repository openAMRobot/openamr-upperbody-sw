#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# A7 regression, headless: G01, G02 (Gazebo), M01 (mock, every lift stop), M02 (Gazebo, baseline, lift at L1000), M03 (ACM sampling), M04 (refused collision goal),
# the M05 gripper-sign and M06 limit-sensitivity checks this step proposes, and the static tests of the SRDF and the MoveIt configuration.
# Writes <workspace>/evidence/<UTC timestamp>_a7.json (openamr.evidence.v1, profile a7, evidence_class simulation).
#
#   source /opt/ros/jazzy/setup.bash && source <workspace>/install/setup.bash
#   ros2 run openamrobot_upperbody_bringup run_a7_regression.sh        # or run this file directly
#
# Exit status: 1 if any check is FAIL (and nothing else: a check that cannot run is a FAIL, not a skip); 3 if every check passed but the tree is dirty:
# the results print and NO evidence file is written (a report describes a commit and a dirty tree is none; A7 evidence from an uncommitted tree is
# not evidence, inherited from P08); 2 if the evidence cannot be assembled; else 0. V01 stays WARN: it needs a person (docs/v01_visual_check.md).
#
# Run `colcon build` first: the tests and this report describe the INSTALLED packages (I01 fails if they are stale). It takes about fifteen minutes:
# the Gazebo tests start a headless server each, M01 starts the mock stack four times, M03 samples 4 x 2000 states. Every test picks its own
# ROS_DOMAIN_ID (and Gazebo partition), so a simulator or a stack you have open is neither seen nor stopped by it.
# OPENAMR_EVIDENCE_DIR overrides where evidence goes; OPENAMR_WS the workspace root; A7_M01_STOPS (comma separated) reduces M01 to some lift stops, which
# the report then states (with the reason recorded).
set -u

BRINGUP=openamrobot_upperbody_bringup
GAZEBO=openamrobot_upperbody_gazebo
MOVEIT=openamrobot_upperbody_moveit
DESC=openamrobot_upperbody_description
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

WS="$(workspace_root)" || { echo "run_a7_regression: cannot find the workspace (no ancestor with install/ and src/); set OPENAMR_WS" >&2; exit 2; }
for tool in colcon gz ros2 python3 git; do
  command -v "$tool" >/dev/null 2>&1 || { echo "run_a7_regression: '$tool' not on PATH: source /opt/ros/jazzy/setup.bash and $WS/install/setup.bash first" >&2; exit 2; }
done
for pkg in $DESC $GAZEBO $MOVEIT $BRINGUP; do
  python3 -c "import ament_index_python.packages as p; p.get_package_share_directory('$pkg')" 2>/dev/null \
    || { echo "run_a7_regression: package $pkg is not on the ament path: source $WS/install/setup.bash" >&2; exit 2; }
done
BUILD_EVIDENCE="$(python3 -c "from ament_index_python.packages import get_package_prefix as g; print(g('$BRINGUP') + '/lib/$BRINGUP/build_a7_evidence.py')")"
[ -f "$BUILD_EVIDENCE" ] || { echo "run_a7_regression: $BUILD_EVIDENCE is missing: colcon build $BRINGUP" >&2; exit 2; }

# Say at the start, not after fifteen minutes, that this run cannot be recorded. It still runs: someone iterating before a commit wants the results.
REPO="$WS/src/openamr-upperbody-sw"
if [ -n "$(git -C "$REPO" status --porcelain --untracked-files=all 2>/dev/null | grep -v '__pycache__/\|\.pyc$')" ]; then
  echo "== NOTE: $REPO has uncommitted changes. The tests will run and print, but NO evidence file will be written (exit 3 if nothing fails)."
fi

export PYTHONDONTWRITEBYTECODE=1        # compiled caches would make the checkout look dirty
export OPENAMR_A7=1                     # un-gates M01 and M02, which skip otherwise so that verify_a6.sh never starts them
START="$(date +%s.%N)"

echo "== run_a7_regression: workspace $WS"
EVIDENCE_DIR="$(python3 "$BUILD_EVIDENCE" --clear)" || exit 2
echo "== evidence directory: $EVIDENCE_DIR (previous G*/M* sidecars cleared)"
for pkg in $GAZEBO $MOVEIT $BRINGUP; do rm -f "$WS/build/$pkg/test_results/$pkg/"*.xunit.xml 2>/dev/null; done

run_tests() {  # <package> <step name> : runs one package's tests, never aborts the script: a failing test is a FAIL in the report
  echo; echo "== colcon test ($1)"
  ( cd "$WS" && colcon test --packages-select "$1" --event-handlers console_cohesion+ )
  local rc=$?
  ( cd "$WS" && colcon test-result --test-result-base "build/$1" --verbose 2>&1 | tail -n 25 )
  # `colcon test` exits 0 whatever its tests did (demonstrated in P12A: a failing test still gave 0); `colcon test-result` is the command that
  # exits 1 for a failing or erroring test, so its status is part of the step's. The report does not rely on either alone: it reads every
  # result file and checks that every registered test left one.
  ( cd "$WS" && colcon test-result --test-result-base "build/$1" >/dev/null 2>&1 ) || rc=1
  eval "RC_$2=$rc"
}
run_tests $GAZEBO gazebo       # G01, G02
run_tests $MOVEIT moveit       # static tests, M03, M04, M05, M06
run_tests $BRINGUP bringup     # L01, M01 x lift stops, M02

echo; echo "== evidence report"
python3 "$BUILD_EVIDENCE" --since "$START" \
  --step "colcon_test_gazebo=$RC_gazebo" --step "colcon_test_moveit=$RC_moveit" --step "colcon_test_bringup=$RC_bringup"
exit $?
