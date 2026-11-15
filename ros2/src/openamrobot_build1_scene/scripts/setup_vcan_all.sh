#!/bin/sh
# SPDX-License-Identifier: MIT
# The five virtual CAN interfaces of the Build 1 scene.
#   vcan_drive vcan_bms vcan_aux   classic CAN (MTU 16), the base buses of openamrobot_sil
#   vcan_arm_l vcan_arm_r          CAN FD (MTU 72), the arm buses of openamrobot_openarm_emulator
# Usage: setup_vcan_all.sh [--delete]
# Needs root or passwordless sudo, iproute2 and the vcan kernel module
# (Ubuntu: sudo apt install linux-modules-extra-$(uname -r)).
# Exit 0: all five are up with the MTU above. Exit 3 with a BLOCKED line: they cannot be had here.
BASE="vcan_drive vcan_bms vcan_aux"
ARMS="vcan_arm_l vcan_arm_r"
SUDO=""
[ "$(id -u)" = "0" ] || SUDO="sudo -n"
blocked() { echo "BLOCKED: $*" >&2; exit 3; }
command -v ip >/dev/null 2>&1 || blocked "the ip command is missing (install iproute2)"

if [ "$1" = "--delete" ]; then
  for iface in $BASE $ARMS; do
    ip link show "$iface" >/dev/null 2>&1 && $SUDO ip link delete "$iface"
  done
  exit 0
fi

$SUDO modprobe vcan >/dev/null 2>&1
for iface in $BASE $ARMS; do
  if ! ip link show "$iface" >/dev/null 2>&1; then
    $SUDO ip link add dev "$iface" type vcan 2>/dev/null \
      || blocked "cannot create $iface (vcan module missing or no permission; on Ubuntu install linux-modules-extra-$(uname -r))"
  fi
done
for iface in $ARMS; do
  if [ "$(cat "/sys/class/net/$iface/mtu")" != "72" ]; then
    $SUDO ip link set "$iface" down
    $SUDO ip link set "$iface" mtu 72 || blocked "cannot set MTU 72 on $iface: this kernel's vcan has no CAN FD"
  fi
done
for iface in $BASE $ARMS; do
  $SUDO ip link set "$iface" up || blocked "cannot bring $iface up"
  echo "$iface: up, mtu $(cat "/sys/class/net/$iface/mtu")"
done
for iface in $ARMS; do
  [ "$(cat "/sys/class/net/$iface/mtu")" = "72" ] || blocked "$iface has MTU $(cat "/sys/class/net/$iface/mtu"), CAN FD needs 72"
done
echo "CAN_FD=yes on $ARMS"
