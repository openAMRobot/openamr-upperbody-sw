# SPDX-License-Identifier: MIT
"""Probe a rosbridge_websocket instance that is meant to be read only, next to a running simulation.

    python3 rosbridge_readonly_probe.py ws://127.0.0.1:9091 SECONDS

Subscribes to the status topics an operator view would read, counts what arrives for SECONDS, then tries what a
read-only bridge must refuse: advertise and publish on /control_room/rosbridge_probe (a topic nothing reads; never a
command topic), a service call (/rosapi/topics, read only itself) and an action goal on /control_room/rosbridge_probe
_action. Prints one JSON object. Whether the refused publish created a ROS publisher is checked by the caller from the
ROS graph (ros2 topic info -v). Development tool for .github/workflows/control-room-e2e.yml; not installed.
"""
import asyncio
import json
import sys

SUBSCRIBE = (("/navigation/status", "openamr_nav_msgs/msg/NavigationStatus"),
             ("/baseline1/mission_status", "std_msgs/msg/String"),      # Proposed (#82)
             ("/sil/bms/battery_state", "sensor_msgs/msg/BatteryState"),
             ("/diagnostics", "diagnostic_msgs/msg/DiagnosticArray"),
             ("/safety/stop_lock", "std_msgs/msg/Bool"),
             ("/battery_status", "std_msgs/msg/Float32"),
             ("/cmd_vel", "geometry_msgs/msg/Twist"))     # outside the subscribe glob: must stay silent
PROBE_TOPIC = "/control_room/rosbridge_probe"


async def probe(url, seconds):
    import websockets
    out = {"url": url, "received": {}, "samples": {}, "other_ops": []}
    async with websockets.connect(url, max_size=2 ** 24) as ws:
        for topic, kind in SUBSCRIBE:
            await ws.send(json.dumps({"op": "subscribe", "topic": topic, "type": kind, "id": f"sub:{topic}"}))

        async def listen(duration):
            end = asyncio.get_event_loop().time() + duration
            while True:
                left = end - asyncio.get_event_loop().time()
                if left <= 0:
                    return
                try:
                    raw = await asyncio.wait_for(ws.recv(), left)
                except asyncio.TimeoutError:
                    return
                msg = json.loads(raw)
                if msg.get("op") == "publish":
                    out["received"][msg["topic"]] = out["received"].get(msg["topic"], 0) + 1
                    out["samples"].setdefault(msg["topic"], json.dumps(msg.get("msg"))[:600])
                else:
                    out["other_ops"].append(json.dumps(msg)[:400])
        await listen(seconds)
        await ws.send(json.dumps({"op": "advertise", "topic": PROBE_TOPIC, "type": "std_msgs/msg/String", "id": "adv"}))
        for i in range(3):
            await ws.send(json.dumps({"op": "publish", "topic": PROBE_TOPIC, "msg": {"data": f"read-only probe {i}"}}))
        await ws.send(json.dumps({"op": "call_service", "service": "/rosapi/topics", "id": "srv", "args": {}}))
        await ws.send(json.dumps({"op": "send_action_goal", "action": PROBE_TOPIC + "_action",
                                  "action_type": "nav2_msgs/action/BackUp", "id": "act", "args": {}}))
        before = len(out["other_ops"])
        await listen(8.0)
        out["after_write_attempts"] = out["other_ops"][before:]
    out["write_results"] = {"service_call_succeeded": any('"service_response"' in o and '"srv"' in o and '"result": true' in o
                                               for o in out["after_write_attempts"])}
    return out


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    result = asyncio.run(probe(argv[0], float(argv[1]) if len(argv) > 1 else 30.0))
    print(json.dumps(result, indent=1, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
