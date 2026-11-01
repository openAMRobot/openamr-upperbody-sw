#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# PROPOSE the collision matrix for config/openamrobot.srdf: derive it from the model (scripts/derive_acm.py) and print what differs from the contract.
#
#   source /opt/ros/jazzy/setup.bash && source <workspace>/install/setup.bash
#   ros2 run openamrobot_upperbody_moveit regenerate_acm.sh [--base-plate bp000] [--write]
#
# THE CHECKED-IN SRDF IS THE CONTRACT. Since P12A the matrix is DERIVED, not sampled: a pair is disabled for exactly three reasons (declared by
# upstream, `Adjacent`, `ConstantTransform`) and every other pair is enabled, so nothing depends on a random generator and regenerating gives the
# same matrix every time (test_acm_generation.py runs it twice and requires the contract back). Before P12A the matrix came from the Setup
# Assistant's collisions_updater, which has no seed option and gave a different set of `Never` pairs on every run; it is no longer used.
# Without --write the SRDF and its record are left alone and a proposal is written to a temporary file; with --write the contract is replaced (the
# record, with the new fingerprint, is rewritten too): that is a decision a reviewer has to see in the diff. Verification of the contract is
# scripts/validate_acm.py (M03).
set -eu
HERE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"      # readlink: under --symlink-install this is the install/ link
for tool in ros2 python3; do
  command -v "$tool" >/dev/null 2>&1 || { echo "regenerate_acm: '$tool' not on PATH: source /opt/ros/jazzy/setup.bash and the workspace install/setup.bash" >&2; exit 2; }
done
exec python3 "$HERE/derive_acm.py" "$@"
