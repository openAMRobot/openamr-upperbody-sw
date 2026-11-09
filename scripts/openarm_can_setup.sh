#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# SocketCAN FD bring-up for the OpenArm 2.0 arms: one CAN FD interface per arm, 1 Mbit/s nominal, 5 Mbit/s data,
# sample points 0.75 / 0.75, dsjw 2 (the defaults of enactic/openarm_can's `openarm-can-cli can_configure`), restart-ms 0
# so that a bus-off stays visible instead of being restarted behind the controller's back.
#
# Needs root (ip link set). Run it before the controllers start: the hardware interface refuses to activate on a link
# that is down. It changes nothing but the named interfaces.
#
#   scripts/openarm_can_setup.sh                       both arms: left on can1, right on can0 (the xacro defaults)
#   scripts/openarm_can_setup.sh --arm left            one arm only (first power-up)
#   scripts/openarm_can_setup.sh --left can3 --right can2
#   scripts/openarm_can_setup.sh --check               verify only, change nothing
#   scripts/openarm_can_setup.sh --down                take the interfaces down
#   scripts/openarm_can_setup.sh --dry-run             print the ip commands, run nothing
#
# Environment: IP overrides the ip command (tests use it).
set -euo pipefail

LEFT_IF=can1
RIGHT_IF=can0
ARM=both
BITRATE=1000000
DBITRATE=5000000
SAMPLE_POINT=0.75
DSAMPLE_POINT=0.75
DSJW=2
# Software choice, not a hardware fact. The kernel's CAN default is 10 frames; one control cycle sends 8 frames per arm.
# TODO(Om) confirm on the bench.
TXQUEUELEN=64
MODE=up
DRY=0
IP_CMD="${IP:-ip}"

usage() { sed -n '3,19p' "$0" | sed 's/^# \{0,1\}//'; }
die() { echo "openarm_can_setup: error: $*" >&2; exit 1; }

while [ $# -gt 0 ]; do
  case "$1" in
    --left) [ $# -ge 2 ] || die "--left needs an interface name"; LEFT_IF=$2; shift 2 ;;
    --right) [ $# -ge 2 ] || die "--right needs an interface name"; RIGHT_IF=$2; shift 2 ;;
    --arm) [ $# -ge 2 ] || die "--arm needs left, right or both"; ARM=$2; shift 2 ;;
    --bitrate) [ $# -ge 2 ] || die "--bitrate needs a value"; BITRATE=$2; shift 2 ;;
    --dbitrate) [ $# -ge 2 ] || die "--dbitrate needs a value"; DBITRATE=$2; shift 2 ;;
    --txqueuelen) [ $# -ge 2 ] || die "--txqueuelen needs a value"; TXQUEUELEN=$2; shift 2 ;;
    --check) MODE=check; shift ;;
    --down) MODE=down; shift ;;
    --dry-run) DRY=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown argument '$1' (try --help)" ;;
  esac
done

valid_if() { [[ "$1" =~ ^[A-Za-z0-9_.-]{1,15}$ ]]; }
is_uint() { [[ "$1" =~ ^[0-9]+$ ]] && [ "$1" -gt 0 ]; }

case "$ARM" in left|right|both) ;; *) die "--arm must be left, right or both, not '$ARM'" ;; esac
valid_if "$LEFT_IF" || die "'$LEFT_IF' is not a valid interface name"
valid_if "$RIGHT_IF" || die "'$RIGHT_IF' is not a valid interface name"
for v in "$BITRATE" "$DBITRATE" "$TXQUEUELEN"; do is_uint "$v" || die "'$v' is not a positive integer"; done
[ "$DBITRATE" -ge "$BITRATE" ] || die "data bitrate $DBITRATE is below the nominal bitrate $BITRATE"

IFACES=()
case "$ARM" in
  left) IFACES=("$LEFT_IF") ;;
  right) IFACES=("$RIGHT_IF") ;;
  both)
    [ "$LEFT_IF" != "$RIGHT_IF" ] || die "left and right are both '$LEFT_IF': each arm needs its own CAN FD bus"
    IFACES=("$LEFT_IF" "$RIGHT_IF") ;;
esac

if [ "$DRY" -eq 0 ]; then
  command -v "$IP_CMD" >/dev/null 2>&1 || die "'$IP_CMD' not found (install iproute2)"
  if [ "$MODE" != check ] && [ "$(id -u)" -ne 0 ]; then
    die "changing a CAN interface needs root: run with sudo (or use --check, which does not)"
  fi
fi

run() {
  if [ "$DRY" -eq 1 ]; then echo "+ $IP_CMD $*"; else "$IP_CMD" "$@"; fi
}

details() { "$IP_CMD" -details link show "$1" 2>&1; }

# Fails with a message when the interface is missing, not a CAN device, or does not match the setup.
verify() {
  local iface=$1 out
  out=$(details "$iface") || die "interface '$iface' not found; ip says: $out"
  grep -q "link/can" <<<"$out" || die "'$iface' is not a CAN interface"
  grep -q "can <" <<<"$out" || grep -q "can state" <<<"$out" || die "'$iface' is not a SocketCAN device"
  grep -Eq "<[^>]*\bFD\b" <<<"$out" || die "'$iface' is not in CAN FD mode (no FD flag): $(grep -m1 'can ' <<<"$out")"
  grep -Eq "\bbitrate $BITRATE\b" <<<"$out" || die "'$iface' nominal bitrate is not $BITRATE"
  grep -Eq "\bdbitrate $DBITRATE\b" <<<"$out" || die "'$iface' data bitrate is not $DBITRATE (does the controller or adapter support CAN FD at that rate?)"
  grep -Eq "restart-ms 0\b" <<<"$out" || die "'$iface' restart-ms is not 0: a bus-off would be restarted silently"
  grep -Eq "<[^>]*\bUP\b" <<<"$out" || die "'$iface' is not UP"
  if grep -Eq "state (BUS-OFF|STOPPED)" <<<"$out"; then
    die "'$iface' controller state is $(grep -Eo 'can <[^>]*> state [A-Z-]+' <<<"$out" | head -1 | awk '{print $NF}'); check wiring, 120 ohm termination at both ends, and that the motors run at the same bitrate"
  fi
  echo "ok: $iface CAN FD $BITRATE/$DBITRATE, restart-ms 0, up"
}

for iface in "${IFACES[@]}"; do
  case "$MODE" in
    down)
      run link set "$iface" down
      ;;
    check)
      verify "$iface"
      ;;
    up)
      if [ "$DRY" -eq 0 ]; then
        details "$iface" >/dev/null || die "interface '$iface' not found (ip link show; USB adapters enumerate in plug order, check which one is which arm)"
        grep -q "link/can" <<<"$(details "$iface")" || die "'$iface' is not a CAN interface"
      fi
      run link set "$iface" down
      run link set "$iface" type can bitrate "$BITRATE" sample-point "$SAMPLE_POINT" \
        dbitrate "$DBITRATE" dsample-point "$DSAMPLE_POINT" dsjw "$DSJW" fd on restart-ms 0 berr-reporting on
      run link set "$iface" txqueuelen "$TXQUEUELEN"
      run link set "$iface" up
      [ "$DRY" -eq 1 ] || verify "$iface"
      ;;
  esac
done
