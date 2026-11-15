# Traces for the replay hook

Synthetic traces in `candump -L` format. Written by hand for the tests. No trace here was recorded on a robot.

| File | Bus | What it holds |
|---|---|---|
| `synthetic_bms_26v4_55pct.log` | `vcan_bms` | Daly answers 0x90, 0x93 and 0x98 from BMS address 0x01 to host 0x40, every 10 ms for 0.1 s. Pack voltage 26.4 V, current 0 A, state of charge 55.0 %. The frame layout is the one `openamrobot_sil` decodes. That layout follows a 2019 protocol copy and is not confirmed for the delivered unit. |
| `synthetic_arm_fd.log` | `vcan_arm_l` | Six CAN FD frames with bit rate switch, data lengths 8, 12, 16, 24, 48 and 64 bytes, and one classic frame. Arbitrary payloads. Not motor traffic. |

Replay a trace in place of an emulator:

```bash
ros2 launch openamrobot_build1_scene build1.launch.py replay_bms:=$(ros2 pkg prefix openamrobot_build1_scene)/share/openamrobot_build1_scene/traces/synthetic_bms_26v4_55pct.log
```
