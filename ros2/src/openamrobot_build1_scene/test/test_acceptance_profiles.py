# SPDX-License-Identifier: MIT
"""The two acceptance profiles (Alex, 18 Nov 2026): ground truth only through the declared sim-oracle profile."""
import os

WF = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", ".github", "workflows", "baseline1-acceptance.yml")


def wf():
    with open(WF, encoding="utf-8") as f:
        return f.read()


def test_profiles_are_explicit_and_map_to_the_pose_source():
    s = wf()
    assert "options: ['sim-oracle', 'perception']" in s
    assert ("BUILD1_PLANNING_POSE_SOURCE: ${{ (github.event.inputs.profile || 'sim-oracle') == 'sim-oracle' "
            "&& 'ground_truth' || 'map' }}") in s


def test_every_run_declares_profile_and_pose_source():
    s = wf()
    assert 'echo "acceptance profile: ${BASELINE1_PROFILE}, pose_source: ${BUILD1_PLANNING_POSE_SOURCE}' in s
    assert "SIMULATION ORACLE: integration evidence, not autonomous perception" in s
    assert 'Acceptance profile: ${BASELINE1_PROFILE}, pose_source: ${BUILD1_PLANNING_POSE_SOURCE}" >> "$GITHUB_STEP_SUMMARY"' in s
