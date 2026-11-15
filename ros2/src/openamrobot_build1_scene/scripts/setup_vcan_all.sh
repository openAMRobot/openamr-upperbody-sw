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
# A new vcan interface takes the kernel's default MTU. On recent kernels that is 2060 (CAN XL), on older ones 16.
# Neither is assumed: the base buses are set to classic CAN and the arm buses to CAN FD.
set_mtu() {
  if [ "$(cat "/sys/class/net/$1/mtu")" != "$2" ]; then
    $SUDO ip link set "$1" down
    $SUDO ip link set "$1" mtu "$2" || blocked "cannot set MTU $2 on $1 ($3)"
  fi
}
for iface in $BASE; do set_mtu "$iface" 16 "classic CAN"; done
for iface in $ARMS; do set_mtu "$iface" 72 "this kernel's vcan has no CAN FD"; done
for iface in $BASE $ARMS; do
  $SUDO ip link set "$iface" up || blocked "cannot bring $iface up"
  echo "$iface: up, mtu $(cat "/sys/class/net/$iface/mtu")"
done
for iface in $BASE; do
  [ "$(cat "/sys/class/net/$iface/mtu")" = "16" ] || blocked "$iface has MTU $(cat "/sys/class/net/$iface/mtu"), classic CAN is 16"
done
for iface in $ARMS; do
  [ "$(cat "/sys/class/net/$iface/mtu")" = "72" ] || blocked "$iface has MTU $(cat "/sys/class/net/$iface/mtu"), CAN FD needs 72"
done
echo "CAN_FD=yes on $ARMS"
