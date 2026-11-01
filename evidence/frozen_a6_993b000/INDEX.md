# Frozen expansion set: A6 at `993b000`

The 14 expansions of `combined_robot.xacro` that A6's contract matrix covers, each with its configuration id, so the
robot description and its identity can be read without building the workspace.

## What it froze (read this first)

This is the **fixed-mast** configuration: <!-- decision-allow: LIFT historical: this frozen set records the superseded fixed-mast model -->
one MISUMI HFS6-60120 mast <!-- decision-allow: MAST-TOP-HEIGHT historical: the superseded mast of the frozen set -->
selected by `mast_id` <!-- decision-allow: CONFIG-IDS historical: the frozen set's config ids were computed with mast_id -->
(`mast_1300`, `mast_1350` <!-- decision-allow: MAST-POSITIONS historical: the superseded mast positions of the frozen set -->
the release configuration, `mast_1400`, `mast_1450`), arms on a plate at the mast top. That is P-03 revision 18.2. <!-- decision-allow: MAST-POSITIONS historical: the superseded mast positions of the frozen set -->

It is **superseded** by P-03 revision 18.9 item 18 (a DOLD Hexalift column, arms mounted on its carriage) and by Alex
Reznichenko's `option-b-upperbody-mount` branch (`6707a87`, 8 Oct), which provides the upper-body geometry and the
`openarm_{left,right}_base_link` mount frames. The mast, the plate, the arm-mount poses and the mast mass in these files
are **not** the robot that will be built, and the configuration ids are the ids of this superseded model. What carries
forward is the structure: the frame contract, the two-call arm composition, how a configuration id is formed, and the
`ros2_control` profiles.

### Two defects in every file, found by looking at it (V01)

- **V01-F1, the mast section is probably rotated 90°.** The mast box is `0.060 x 0.120` (x fore-aft, y lateral). Option B
  orients the same HFS6-60120 cross-section (its 340 mm `arm_profile`, not a full-height mast) 120 mm fore-aft and 60 mm <!-- decision-allow: MAST-TOP-HEIGHT negative: the arm profile is not a full-height mast -->
  across, the stiff direction against the arms' forward cantilever; so "probably".
  `mast_section = [0.060, 0.120]` is a right value; its axis assignment was only a prose note ("orientation
  unconfirmed") and no test asserted it. It is coupled to `left/right_arm_mount_pose` (y = +/-0.060), which sit on the
  side faces of the 120 mm width, so it cannot be turned in isolation.
- **V01-F2, the mast mass is implausible.** `mast_mass = 3.0` kg (placeholder) for a 1.2365 m profile: about 337 kg/m3
  over the bounding box. A hollow aluminium extrusion fills perhaps a quarter to a half of its bounding box, which at
  2700 kg/m3 is of the order of 700 to 1500 kg/m3: an estimate, not a datasheet value. That makes 3.0 kg low by roughly
  2.5 to 4 times; the real figure needs the MISUMI datasheet. The modelled total of 21.96 kg is light by the difference.

Neither is fixed here: both die with `mast.xacro` when option B is adopted. See `docs/v01_visual_check.md`.

## Provenance

| | |
|---|---|
| source | `openamr-upperbody-sw` @ `993b000bcc3d77994b84b35846cfe7da784c6923` (`feat/a6-canonical-description`, the head of the A6 branch) |
| datums revision | `2026-10-08.e` (`inputs/datums.yaml`, `inputs/mast_ids.yaml` are the files at that commit) |
| `openarm_description` | `14ff67b638ff1c738a1b9a6be8aaa5ce5ed2c831`, the commit the id hashes in |
| platform base | `openamr-platform-sw` @ `12de3ef9503073ab41fd4118f3a018809ad51c4e` (`openamrobot_description`, included unchanged; part of every file, not a separate input to the id) |
| also pinned | `openarm_ros2` @ `4e837e1d0dae692ff67b560b69d8d281d7a8d4ed` (control and MoveIt configuration; not an input to these files) |
| produced | `make_frozen_set.py`, from a clean `git archive 993b000` tree; the ids were then compared with A6's own report |
| cross-check | all 14 ids equal those in `20261006T164342Z_dev.json` (`verify_a6.sh` at `993b000`, tree clean, 20 PASS / 6 WARN / 0 FAIL) |
| evidence class | **static**: xacro expansion and hashing. Nothing was launched, simulated or measured |

## The files

`config_id` is the project's id (ARCHITECTURE.md section 7): `sha256` over the canonical (C14N, comments dropped)
expansion, the three arguments, the datums revision and the `openarm_description` commit. It identifies inputs; it says
nothing about whether the robot is right. Links / joints and modelled mass are read from the file (mass is the sum of
`<inertial>` masses: the description's own total, not the robot's weight).

| configuration | file | config_id | links / joints | modelled kg | arm hardware plugin |
|---|---|---|---|---|---|
| `mast_1300/openarm/mock` | [mast_1300__openarm__mock.urdf](expansions/mast_1300__openarm__mock.urdf) | `sha256:c9aed2ad17e8b32571cc5d48e01f53a937ba79fcc1f1d2d8f605d7c672a5ff9e` | 46 / 45 | 21.96 | mock_components/GenericSystem <!-- decision-allow: MAST-POSITIONS historical: frozen expansion of the superseded mast model --> |
| `mast_1300/so101/mock` | [mast_1300__so101__mock.urdf](expansions/mast_1300__so101__mock.urdf) | `sha256:bdad284be4b34dfd05bd70cbe5c390acc48149ef078d44c219ecc97b31c27b77` | 17 / 16 | 9.70 | none <!-- decision-allow: MAST-POSITIONS historical: frozen expansion of the superseded mast model --> |
| `mast_1300/none/mock` | [mast_1300__none__mock.urdf](expansions/mast_1300__none__mock.urdf) | `sha256:d1cb6ee23be0c4ab342237532367909da2aebc383a7fd4ea8ee85744560019e5` | 16 / 15 | 9.70 | none <!-- decision-allow: MAST-POSITIONS historical: frozen expansion of the superseded mast model --> |
| `mast_1350/openarm/mock` | [mast_1350__openarm__mock.urdf](expansions/mast_1350__openarm__mock.urdf) | `sha256:db76c29a6607137a55b67168e35ce571248486404d57799037c4061447829706` | 46 / 45 | 21.96 | mock_components/GenericSystem |
| `mast_1350/so101/mock` | [mast_1350__so101__mock.urdf](expansions/mast_1350__so101__mock.urdf) | `sha256:4c822ea2daff9e4fb5c948c2660d5bc73572be38df5f67b48608940f000695a7` | 17 / 16 | 9.70 | none |
| `mast_1350/none/mock` | [mast_1350__none__mock.urdf](expansions/mast_1350__none__mock.urdf) | `sha256:b821aecb3ba353345a0de0200c34634806f3eaa383fe8d9cdb9549fd10ea2216` | 16 / 15 | 9.70 | none |
| `mast_1400/openarm/mock` | [mast_1400__openarm__mock.urdf](expansions/mast_1400__openarm__mock.urdf) | `sha256:b63b9985547bd27e1ac1d69e11651fd84b1d518366f5296df39c9e2737175b22` | 46 / 45 | 21.96 | mock_components/GenericSystem <!-- decision-allow: MAST-POSITIONS historical: frozen expansion of the superseded mast model --> |
| `mast_1400/so101/mock` | [mast_1400__so101__mock.urdf](expansions/mast_1400__so101__mock.urdf) | `sha256:bc2a52b0ffa56c1a7bfb584f1b86c19e2efb17eb0e11769b8a2c2709f44f2b48` | 17 / 16 | 9.70 | none <!-- decision-allow: MAST-POSITIONS historical: frozen expansion of the superseded mast model --> |
| `mast_1400/none/mock` | [mast_1400__none__mock.urdf](expansions/mast_1400__none__mock.urdf) | `sha256:c4d50412792c886204de2c7ae795cb229885038485d403362305d7b4cfcaff19` | 16 / 15 | 9.70 | none <!-- decision-allow: MAST-POSITIONS historical: frozen expansion of the superseded mast model --> |
| `mast_1450/openarm/mock` | [mast_1450__openarm__mock.urdf](expansions/mast_1450__openarm__mock.urdf) | `sha256:2f0be25f3acd50e6e262f3b0934f0bc9014d7cd03ea0fcdf87cd7986dd468d17` | 46 / 45 | 21.96 | mock_components/GenericSystem <!-- decision-allow: MAST-POSITIONS historical: frozen expansion of the superseded mast model --> |
| `mast_1450/so101/mock` | [mast_1450__so101__mock.urdf](expansions/mast_1450__so101__mock.urdf) | `sha256:d946f623e075b550cbcc34f7bec588f92afbd458667a9db50807380eb58991bf` | 17 / 16 | 9.70 | none <!-- decision-allow: MAST-POSITIONS historical: frozen expansion of the superseded mast model --> |
| `mast_1450/none/mock` | [mast_1450__none__mock.urdf](expansions/mast_1450__none__mock.urdf) | `sha256:54796490fc1ff82f0588ad5df72bc4898a9e0607fd1c8b605e3e8c54fd6e6cfc` | 16 / 15 | 9.70 | none <!-- decision-allow: MAST-POSITIONS historical: frozen expansion of the superseded mast model --> |
| `mast_1350/openarm/gazebo` | [mast_1350__openarm__gazebo.urdf](expansions/mast_1350__openarm__gazebo.urdf) | `sha256:154721502f85574415167048723990c862440db081c8efac2f9a1f6e0a400ef4` | 46 / 45 | 21.96 | gz_ros2_control/GazeboSimSystem |
| `mast_1350/openarm/real` | [mast_1350__openarm__real.urdf](expansions/mast_1350__openarm__real.urdf) | `sha256:18b1f1c0024cef651fbbcedb0d826dd52da5372d589fa1133ced8a65d6f301db` | 46 / 45 | 21.96 | openarm_hardware/OpenArmHW |

14 files, 476 KiB. **Size:** plain UTF-8, not gzipped or bundled: under half a megabyte, readable and diffable
on GitHub, and each file is exactly the text its id was computed from.

### The one id that differs on the A7 branch

`mast_1350/openarm/gazebo` is `sha256:154721502f85574415167048723990c862440db081c8efac2f9a1f6e0a400ef4` at `993b000` and `sha256:ad480a8cc55f42538dda586c604d5f6443ed8f24302750ef342b193f36aa8fe5` at the A7
head (`a488743`). Cause: A7 adds the `gz_ros2_control` model plugin to the gazebo profile (a
`<gazebo><plugin>` block), which A6's expansion lacks. The other 13 ids are identical on both branches. The A7 variant is
kept apart so the two are not mixed up:

| configuration | file | config_id | links / joints | modelled kg | arm hardware plugin |
|---|---|---|---|---|---|
| `mast_1350/openarm/gazebo` | [mast_1350__openarm__gazebo.urdf](a7_variant/mast_1350__openarm__gazebo.urdf) | `sha256:ad480a8cc55f42538dda586c604d5f6443ed8f24302750ef342b193f36aa8fe5` | 46 / 45 | 21.96 | gz_ros2_control/GazeboSimSystem + `gz_ros2_control` model plugin |

## Verify without our workspace

    python3 verify_config_ids.py

Python 3.8+ standard library only. It recomputes every id from the file and compares it with `index.json` and with the id
in the file's own header comment. The header comment replaces xacro's banner (which names a local path); comments are
dropped before hashing, so it does not change an id.

## What is not here

- **Meshes.** The files reference `package://openamrobot_description/meshes/...` (12 STL, from `openamr-platform-sw`
  @ `12de3ef`) and `package://openarm_description/assets/...` (20 files, STL and DAE, from `openarm_description` @
  `14ff67b`). Kinematics, inertials, collision primitives, joint limits and `ros2_control` blocks are all in the files.
- **The runtime configuration-id producer.** None exists yet. The id is computed offline by `print_config_id.py` and
  recorded in the evidence report; no node publishes it.
- **Controller parameters** (`arm_controllers.yaml`) and the MoveIt configuration: those are not part of the expansion.

## Reproduce

    git archive 993b000 | tar -x -C /tmp/a6                      # in openamr-upperbody-sw
    # put a prefix whose share/openamrobot_upperbody_description is /tmp/a6/ros2/src/openamrobot_upperbody_description
    # FIRST on AMENT_PREFIX_PATH, then:
    python3 make_frozen_set.py --scripts-dir /tmp/a6/ros2/src/openamrobot_upperbody_description/scripts \
        --out OUT --commit 993b000bcc3d77994b84b35846cfe7da784c6923 --branch feat/a6-canonical-description \
        --platform-sw 12de3ef9503073ab41fd4118f3a018809ad51c4e

`index.json` is that script's per-entry output plus the provenance fields above, added by hand.

## Why this is on the A7 branch

A6's head is `993b000` and stays there: PR-1 is that commit. This set was added on top of A7, whose merge base is
`993b000`, so that A6's history is not touched.
