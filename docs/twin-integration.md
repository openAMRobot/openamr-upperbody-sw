# Full robot twin — Proposed integration, 1 November 2026

The twin normally merges official equipment PR #26 (`4316ef09bb6117b12c6a84482538047896b6847c`) into the tested descendant of PR #22. Both histories, including the complete PR #14 feature implementation, remain intact. One canonical combined Xacro consumes platform-sw through `config/upstream_pins.yaml`; OpenArm pins are unchanged. P-03 rev20.2 is in force.

After building and sourcing the workspace, launch explicitly in simulation:

```bash
ROS_DOMAIN_ID=91 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_PARTITION=openamrobot_twin_20261101 \
ros2 launch openamrobot_upperbody_bringup moveit.launch.py sim:=gazebo \
  column:=dold_v4_350 base_plate:=bp000 head_pitch:=hp30 base_tilt:=bt10 \
  physics_engine:=gz-physics-bullet-featherstone-plugin rviz:=true headless:=false
```

Use an isolated container with no robot devices or robot network. Ctrl-C stops this launch. The integration workspace supplies tracked process-group start/stop helpers, an exact dependency image, source bundles and command/evidence records. The old workspace is never modified.

Bullet Featherstone runs the complete base, both arms and both physical mimic grippers. The reproduced DART baseline did not close the grippers. Valid collision surfaces, explicit ideal-floor friction and 0.125 ms physics steps are used with the stock solver. Coefficients and rigid contacts are approximations, not measured materials. Gazebo 8.11 publishes base joint states every physics iteration; an unsupported later-version rate setting is not used.

Initial world placement is derived from the consumed URDF drive/castor support plane and nominal COM: baseline approximately z=0.002004 m, pitch=0.007700 rad, with 1 µm numerical clearance and all joints zero. It supersedes the flat 3 mm placement that failed M02's unchanged 1e-5 rad diagnostic gate. Rear castors remain approximately 4.2 mm clear: all six loaded contacts are not verified, and the approved axle/support/load configuration needs mechanical reconciliation. The 0.400 m track placeholder and approved LiDAR frame remain unchanged; physical Gate A measurements are not copied into 2.0.

Official `upper_body_equipment.xacro` owns item-24 equipment, its fixed/moving frames and portrait report. The twin retains supplier-backed Falcon housing/window and SICK bracket dimensions, including an aperture instead of a solid slab across the sensor. The rod's provisional corner belongs to its rigid link assembly; camera housing/window are additive. Manufacturer envelopes supersede older roadmap placeholders; M-09 placement, tablet pivot and portrait clearance remain provisional. Original docs JSON fixtures stay unchanged. The official exporter reports raw roadmap discrepancies; separately reconciled tests read only the named supplier values from `datums.yaml`, with the original 0.5 mm tolerance. No duplicate constants/model remain.

Supplier masses are counted once on the moving assembly: Getac 0.590 kg, head camera 0.0629 kg and Falcon light upper-bound 0.110 kg. Uniform-box inertias are approximations. The 11.7 kg lift catalogue mass remains on the fixed column pending a delivered fixed/moving split. Legacy base inertials omit documented equipment; the model sum is not as-built gross mass or validated COM.

Equipment/ACM adoption is Proposed under [#23](https://github.com/openAMRobot/openamr-upperbody-sw/issues/23). The integrated derived matrix has 188 justified pairs (20 upstream, 27 derived Adjacent, 141 clear ConstantTransform). Every other pair stays enabled. No permanent collision is waived and no sampler certifies allowances. The original 26 mm castor/chassis regression remains alongside CAD-derived 17.5 mm support checks.

`config/simulation_cameras.yaml` owns the Proposed ideal head/wrist profile under [#24](https://github.com/openAMRobot/openamr-upperbody-sw/issues/24): 640×480, 90-degree horizontal FOV, 10 Hz, 0.02–10 m clip. Producers/bridges consume that file, existing optical frames and Gazebo timestamps. These are monocular ideal viewpoints, not delivered wrist calibration or ZED stereo fidelity. Base RGB-D remains in platform-sw; legacy base camera compatibility is retained.

The task runtime's 16 MiB Fast DDS shared-memory profile avoids reproduced image loss without changing camera specifications/QoS. Its narrow MoveIt 2.12.4 override combines attributed upstream PR #3828's TEM destruction-order fix with a capability-loader lifetime guard. The unmodified/upstream-only runtimes fail the strict teardown regression; the complete repair passes three launches. Clamp shutdown finishes active callbacks before context destruction. Exact runtime source/library hashes must accompany evidence; no crash allowance is added.

The lift has no supplied dynamic actuator in this arm-only controller setup. Static L1000/L1175/L1350 geometry/flag tests are not actuation evidence. No control authority, joint limit, configuration-ID algorithm or safety implementation changed.

Run installed A6/A7, world-truth base motion/stops, real MoveIt arm/gripper execution, mimic link measurements, collisions, sensors/TF and repeated start/stop on the final clean source combination. Preserve failures and distinguish package/mock coverage from complete-robot physics measurements. Final counts, exact heads and artifacts belong in the draft PR. Physical/as-built matching, ideal-simulation motion tolerances and owner adoption remain pending. Optional V01 pictures are bounded diagnostics, never substitutes for A7 evidence.
