#!/usr/bin/env bash
# One M02 run: $1 = tag. Overlay prefixes (colon separated) in $OVERLAYS are put first on AMENT_PREFIX_PATH.
set +u
S=${SCRATCH:-/tmp/m02_bisect}
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
# pre-run hygiene: nothing of an earlier run of mine (domain 101-200, or an m02_/g02_ partition) may still be alive
for d in /proc/[0-9]*; do
  p=${d#/proc/}; [ "$p" = "$$" ] && continue
  e=$(tr '\0' '\n' < $d/environ 2>/dev/null | grep -E '^(ROS_DOMAIN_ID|GZ_PARTITION)=')
  dom=$(echo "$e" | grep ROS_DOMAIN_ID | cut -d= -f2); part=$(echo "$e" | grep GZ_PARTITION | cut -d= -f2)
  if echo "$part" | grep -qE '^(m02|g02)_' || { [ -n "$dom" ] && [ "$dom" -ge 101 ] 2>/dev/null && [ "$dom" -le 200 ] 2>/dev/null; }; then
    echo "PRE-RUN LEFTOVER killed: $p $(tr '\0' ' ' < $d/cmdline | cut -c1-80)" >> $S/m02_prerun_leftovers.txt; kill -KILL $p 2>/dev/null
  fi
done
WS_ROOT=${WS_ROOT:-/home/slash/openamr_ws}; source $WS_ROOT/install/setup.bash >/dev/null 2>&1
[ -n "${OVERLAYS:-}" ] && export AMENT_PREFIX_PATH="$OVERLAYS:$AMENT_PREFIX_PATH"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp OPENAMR_A7=1
export OPENAMR_EVIDENCE_DIR=$S/m02_ev/$1; rm -rf "$OPENAMR_EVIDENCE_DIR"; mkdir -p "$OPENAMR_EVIDENCE_DIR"
cd $WS_ROOT
rm -f $WS_ROOT/build/openamrobot_upperbody_bringup/test_results/openamrobot_upperbody_bringup/*m02*.xunit.xml 2>/dev/null
colcon test --packages-select openamrobot_upperbody_bringup --ctest-args -R m02 --event-handlers console_direct+ > $S/m02_$1.log 2>&1
echo "COLCON_EXIT=$?" >> $S/m02_$1.log
# orphans of this run: recorded, then stopped by pid (only servers whose partition is an m02_ one: ours)
: > $S/m02_$1.leftover
sleep 2
for p in $(pgrep -f "^gz sim"); do
  part=$(tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep '^GZ_PARTITION=m02_')
  [ -n "$part" ] && { echo "$p $part" >> $S/m02_$1.leftover; kill -TERM $p; sleep 3; [ -d /proc/$p ] && kill -KILL $p; }
done
python3 - "$OPENAMR_EVIDENCE_DIR" "$1" <<'PY' >> $S/m02_results.txt
import json,sys,glob
d,tag=sys.argv[1:3]
for f in glob.glob(d+"/M02_*.json"):
    j=json.load(open(f)); c=j["details"]["clamp"]
    print(tag, j["result"], "msgs",c.get("messages"),"act",c.get("activations"),"ep",c.get("episodes"),"max",c.get("max_magnitude_rad"),c.get("max_magnitude_joint"),
          "joints",{k:(v["activations"],v["episodes"],v["max_magnitude"]) for k,v in c.get("joints",{}).items()})
    break
else: print(tag,"NO SIDECAR")
PY
