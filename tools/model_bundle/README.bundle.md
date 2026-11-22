<!-- SPDX-License-Identifier: CC-BY-4.0 -->
# {{BUNDLE}}

The OpenAMRobot 2.0 robot model, version {{VERSION}}: the mobile base, the 350 mm lift and two OpenArm 2.0 arms with their
sensors, as expanded URDF files with every mesh they use. Built by CI from the commits listed below; nothing in it is
edited by hand.

**Simulation and visualisation model, not hardware validation.** The model has been checked by xacro expansion,
`check_urdf` and the file checks of this bundle. Nothing in it has been measured on a built robot, and nothing here is
safety evidence.

## What is in the bundle

| Path | What it is |
|---|---|
| `urdf/*.urdf` | the expanded models, one per configuration (table below) |
| `share/<package>/...` | the mesh files the URDFs reference, at the path their `package://<package>/...` URI names |
| `share/ament_index/...` | empty marker files, so that ROS 2 finds the two mesh packages in this folder |
| `bundle_lock.json` | the lock: the commit of every source repository, the toolchain, the xacro arguments of each URDF, and the path, size, sha256, licence and source of every file |
| `SHA256SUMS` | the sha256 of every file, for `sha256sum -c SHA256SUMS` |
| `LICENSE`, `NOTICE.md`, `licenses/` | licences and notices (below) |

### Configurations

| URDF | Use | xacro arguments | links / joints |
|---|---|---|---|
{{CONFIG_ROWS}}

All other arguments take their defaults: lift column `dold_v4_350`, head camera pitch `hp30`, base camera tilt `bt10`.
The lift is a joint (`lift_joint`, 0 to 0.350 m), so every URDF has the full stroke; the shoulder is 1.000 m above the
floor at lift 0 and 1.350 m at full stroke. `bp000` and `bp050` are the two base-plate positions of the column (the column
axis 0.060 m or 0.110 m ahead of `base_link`).

### Sources

| Source | Repository | Commit |
|---|---|---|
{{SOURCE_ROWS}}

MJCF (MuJoCo model): {{MJCF}}.

## Check the files

```bash
sha256sum -c SHA256SUMS
```

## Load it

The commands below assume Ubuntu 24.04 with ROS 2 Jazzy, a shell in the unpacked bundle folder, and
`source /opt/ros/jazzy/setup.bash`. They are not run by CI: CI checks the expansion and the files, not a viewer.

### RViz

```bash
export AMENT_PREFIX_PATH=$PWD:$AMENT_PREFIX_PATH
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat {{FIRST_URDF}})"
```

In a second shell with the same two lines run first, `ros2 run joint_state_publisher_gui joint_state_publisher_gui`
(sliders for the lift and the arm joints), and in a third, `rviz2`: set the fixed frame to `base_footprint` and add a
RobotModel display on the topic `/robot_description`. The `AMENT_PREFIX_PATH` line is what lets RViz resolve the
`package://` mesh URIs from this folder.

### Gazebo Harmonic

```bash
export GZ_SIM_RESOURCE_PATH=$PWD/share:$GZ_SIM_RESOURCE_PATH
gz sim -r empty.sdf
```

In a second shell with the same export, `ros2 run ros_gz_sim create -file {{GAZEBO_URDF}} -name openamrobot`. The
`*-gazebo` URDFs carry a `gz_ros2_control` block with no controller file, so no controller holds the joints: the arms
and the lift fall under gravity. For a controlled simulation build the repository and use its launch files
(`openamrobot_upperbody_gazebo`).

### MuJoCo

This bundle has no MJCF file (see above). The `*-dynamics` URDFs are the input meant for a MuJoCo export: they carry
the joint damping and friction of the OpenArm joints. MuJoCo's own URDF import has not been tried with this model and is
not a supported path. The OpenArm visual meshes are COLLADA (`.dae`), which MuJoCo does not read natively.

## Limitations

These are known and open. Read them before you rely on a number.

- **Drive track 0.400 m is a placeholder** in the mobile-base description; the decision is open and the 2.0 wheel
  assembly will change it.
- **Castors are fixed joints**: there is no swivel joint in the model.
- **Masses and inertias are design values** from the mass ledger and data sheets. Nothing has been weighed.
- **Joint damping and friction of the arms** come from the upstream OpenArm MuJoCo model. They are not measurements of a
  delivered arm. The rotor inertia (armature) is not in the URDF, because URDF has no element for it.
- **Lift limits** (800 N, 0.018 m/s) are the recorded design values for the DOLD Hexalift V4 column, not measured on
  the robot. The fallback column `tl3_400` has no supplier data and is not in this bundle.
- **Vendor parts on the base are envelopes**: motors, castors, sensors, compute and battery are boxes, cylinders and
  revolved profiles from their data-sheet dimensions, not vendor CAD.
- **Cameras and sensors** are frames and simulation sensors with working values for field of view, resolution and rate.
- **Not modelled**: the hardwired safety chain (E-stops, contactors, brakes, charge inhibit), the hub motor holding
  brakes, the CAN physical layer, battery behaviour, motor current and heat, gear backlash, cable drag, sensor noise.
- **Reality gaps** that are tracked and open: hub motor holding brake not modelled; drive reaction to loss of
  communication emulated from a proposal; wheel velocity unit and range from a filed manual revision; arm driver
  behaviours recorded with an emulator only; castors fixed; drive track placeholder; floor and wheel friction not
  measured; simulation clocks not shared with the emulated devices. Summary and status:
  https://docs.openamrobot.ai/reference/openamrobot-2/digital-twin/status/

## Licences

The expanded URDFs compose files under three licences: the OpenAMRobot model sources (MIT), the CAD-derived base
description (CERN-OHL-P-2.0) and the OpenArm 2.0 description (Apache-2.0, Copyright 2025 Enactic, Inc.). Each mesh keeps
the licence of its source: `NOTICE.md` lists every file by licence, and `bundle_lock.json` gives each file's licence and
the repository, commit and path it was copied from. Documentation in this bundle is CC-BY-4.0. The names OpenAMRobot and
OpenAMR are not licensed by any of these licences.
