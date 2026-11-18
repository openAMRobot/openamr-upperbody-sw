# openamrobot_control_room

Digital Twin Control Room: read-only visual monitoring of the OpenAMRobot 2.0 simulation. No command authority.
Simulation monitoring only: no hardware, physical acceptance or safety evidence.

- `launch/control_room.launch.py`: the monitor nodes, RViz2 and (optional) a subscribe-only foxglove_bridge
- `rviz/control_room.rviz`: view and measure tools only
- `config/foxglove_bridge.yaml`, `config/lichtblick_layout.json`: browser viewing without write paths
- `scripts/control_room_monitor.py`, `scripts/vcan_monitor.py`: summaries on `/control_room/`
- `scripts/mission_report.py`: one static HTML page from a recorded bag and the mission JSON

Usage, displays, the read-only tests and the limitations: [docs/demo/control_room.md](../../../docs/demo/control_room.md).
