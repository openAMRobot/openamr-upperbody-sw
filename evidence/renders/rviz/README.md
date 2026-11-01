# RViz screenshots (P19)

`../rviz_L1000_bp000.png` ... `../rviz_L1350_bp050.png`: **real RViz2** (the MotionPlanning display on the combined model) on the real display
(WSLg), one window capture per lift stop (L1000, L1175, L1350) and base plate (bp000, bp050), with `../rviz_*.json` recording what was read from
`/joint_states` (the lift joint is 0.000 / 0.175 / 0.350 m) and that nothing was left running. They are not the URDF-visual renders of
`render_v01.py`; those are a different tool and stay what they were.

    python3 capture_rviz.py <outdir>      # needs the workspace sourced and a display; about 2 minutes per shot

`capture.launch.py` is the mock MoveIt stack (`moveit.launch.py sim:=mock`) plus an RViz2 with `capture.rviz`, a copy of `moveit.rviz` with one
camera that frames the robot at all three stops. `xgrab.py` captures one window (PIL's root-window grab fails under WSLg). What a person runs is

    ros2 launch openamrobot_upperbody_bringup moveit.launch.py sim:=mock rviz:=true lift_position:=0.175 base_plate:=bp050

What the shots show: the model as `moveit.launch.py` loads it, taken on platform-sw `d619012`. The lift column's two placeholder boxes are drawn
solid red (the URDF gives them no material; I did not investigate whether that is RViz's default for a visual without one). A person has still
not looked at the running simulator, so V01 stays WARN.

**P25 (21 Oct 2026):** the red was the missing material. The three placeholder boxes now have one named material (rgba 0.78 0.64 0.45, tan) and the same
capture at `L1000` / `bp050` draws them tan; the evidence is in `../../p25/` (`rviz/`, `gazebo/`, `colour_compare.json`). The P19 PNGs here are kept as the "before".
