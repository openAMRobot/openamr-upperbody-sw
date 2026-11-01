# W03: named poses that coincide with a joint limit (evidence files only)

`old_body_7daa2ab.json` (the old mast body) and `new_body_HEAD.json` (the lift body at the time of P18) are the output of

    python -m openamrobot_upperbody_moveit.named_poses <srdf> <urdf>

for each body. Both list the same four joints (`openarm_{left,right}_joint4` at `home` and `park`, `openarm_{left,right}_finger_joint1` at `open` and
`closed`; eight poses): they are limits in upstream's joint definitions, in the old body as in the new one.

**This is not a check.** W03 is not an entry of the A7 report, writes no sidecar, and gates nothing, while M07 exists (P19: Alex's M07-as-WARN
on these same four joints stands; if he accepts the withdrawal, W03 can become a check in one commit). The helper has a unit test
(`test_named_poses_on_limits.py`) and nothing else uses it.
