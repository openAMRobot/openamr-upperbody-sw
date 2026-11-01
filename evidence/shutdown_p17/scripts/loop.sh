#!/usr/bin/env bash
cd "$(dirname "$0")"; source /opt/ros/jazzy/setup.bash; source /home/slash/openamr_ws/install/setup.bash
for i in $(seq 1 ${N:-6}); do python3 shutdrv.py ${PFX:-b}$i; done; echo ALLDONE
