#!/usr/bin/env bash
# 12 x (G01, G02, M02), each through colcon test, each with its own evidence dir; records the detector's verdict per run.
set +u
S=/tmp/claude-1000/-home-slash-openamr-ws/7fc564b9-e416-46a0-9c88-0064b2d16549/scratchpad
WS=/home/slash/openamr_ws
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; source $WS/install/setup.bash >/dev/null 2>&1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp OPENAMR_A7=1
leftovers() {   # processes of earlier runs of mine (private partitions or domains 101-200), killed by pid and recorded
  for d in /proc/[0-9]*; do
    p=${d#/proc/}; [ "$p" = "$$" ] && continue
    e=$(tr '\0' '\n' < $d/environ 2>/dev/null | grep -E '^(ROS_DOMAIN_ID|GZ_PARTITION)=')
    part=$(echo "$e" | grep GZ_PARTITION | cut -d= -f2); dom=$(echo "$e" | grep ROS_DOMAIN_ID | cut -d= -f2)
    if echo "$part" | grep -qE '^(g01|g02|m02|m07|sd|effort_probe|st)_' || { [ -n "$dom" ] && [ "$dom" -ge 101 ] 2>/dev/null && [ "$dom" -le 200 ] 2>/dev/null; }; then
      echo "LEFTOVER before $1: $p $(tr '\0' ' ' < $d/cmdline | cut -c1-70)" >> $S/o12/leftovers.txt; kill -KILL $p 2>/dev/null
    fi
  done 2>/dev/null
}
run() {   # $1 tag, $2 package, $3 ctest regex
  leftovers $1
  export OPENAMR_EVIDENCE_DIR=$S/o12/ev_$1; rm -rf $OPENAMR_EVIDENCE_DIR; mkdir -p $OPENAMR_EVIDENCE_DIR
  cd $WS; rm -f build/$2/test_results/$2/*.xunit.xml 2>/dev/null
  t0=$(date +%s)
  colcon test --packages-select $2 --ctest-args -R $3 --event-handlers console_direct+ > $S/o12/$1.log 2>&1
  echo "$1 rc=$? secs=$(( $(date +%s) - t0 ))" >> $S/o12/runs.txt
}
for i in $(seq 1 12); do
  run g01_$i openamrobot_upperbody_gazebo g01
  run g02_$i openamrobot_upperbody_gazebo g02
  run m02_$i openamrobot_upperbody_bringup m02
done
leftovers end
echo ALLDONE >> $S/o12/runs.txt
