# SPDX-License-Identifier: MIT
"""Cut small test fixtures out of one recorded Baseline 1 or grocery run (its uploaded build1_logs directory).

    python3 extract_run_fixtures.py BUILD1_LOGS OUT_DIR --label baseline1_44309817813

Writes OUT_DIR/<label>/:
  candump_<bus>.log   selected candump -L lines of each bus, unchanged (windows named in selection.json)
  messages.json       selected messages of the device, mission and run topics of the bag, with their bag
                      receive time (the recorder runs with --use-sim-time: Gazebo seconds)
  mission.json        the mission log without its trace samples (the trace length is kept)
  bag_info.txt        as recorded
  selection.json      which lines and messages were kept and why

Every value is copied from the recording; nothing is computed or invented here. Development tool: run by
.github/workflows/control-room-evidence.yml, not installed with the package.
"""
import argparse
import json
import os
import sys

BUSES = ("vcan_drive", "vcan_aux", "vcan_bms", "vcan_arm_l", "vcan_arm_r")
JSON_TOPICS = ("/sil/drive/status", "/sil/drive/emulator_state", "/sil/lift/status", "/sil/lift/emulator_state",
               "/sil/bms/status", "/sil/bms/emulator_state", "/sil/fw/status", "/build1/run_info", "/build1/rtf_stats")
ALL_TOPICS = ("/grocery_demo/phase", "/grocery_demo/events", "/sil/lift/command", "/sil/lift/permit",
              "/baseline1/mission_status")
SAMPLES_PER_TOPIC = 40


def frame_id(line):
    try:
        return line.split()[2].split("#")[0].upper()
    except IndexError:
        return None


def payload(line):
    """Hex data of a candump -L line: ID#DATA (classic) or ID##FDATA (CAN FD, F is the flags nibble)."""
    try:
        field = line.split()[2]
    except IndexError:
        return ""
    _, _, rest = field.partition("#")
    return rest[2:].upper() if rest.startswith("#") else rest.upper()


def windows(lines, name):
    """(reason, start, end) index windows of one bus."""
    n = len(lines)
    out = [("first frames", 0, min(n, 120)), ("last frames", max(0, n - 60), n)]
    for k in (1, 2, 3):
        mid = n * k // 4
        out.append((f"window at {k}/4 of the recording", mid, min(n, mid + 80)))
    ids = [frame_id(line) for line in lines]
    if name == "vcan_aux":
        # Each lift command (RPDO1 0x210 with a new sequence byte) with its context, and the first EMCY.
        seen = set()
        for i, (fid, line) in enumerate(zip(ids, lines)):
            if fid == "210":
                data = payload(line)
                if data[:2] not in seen:
                    seen.add(data[:2])
                    out.append((f"lift command seq 0x{data[:2]}", max(0, i - 30), min(n, i + 150)))
            if fid == "090" and "emcy" not in seen:
                seen.add("emcy")
                out.append(("first EMCY", max(0, i - 10), min(n, i + 10)))
    if name == "vcan_drive":
        # The first SDO writes of a nonzero wheel target (60FFh) and the frames after them.
        for i, line in enumerate(lines):
            if ids[i] == "601":
                data = payload(line)
                if data[2:6] == "FF60" and data[8:16] != "00000000":
                    out.append(("first nonzero 60FFh target", max(0, i - 20), min(n, i + 200)))
                    break
    if name in ("vcan_arm_l", "vcan_arm_r"):
        for i, fid in enumerate(ids):
            if fid == "7FF":
                out.append(("first 0x7FF frame", max(0, i - 10), min(n, i + 60)))
                break
        for i, line in enumerate(lines):
            data = payload(line)
            if data.startswith("FFFFFFFFFFFFFF"):
                out.append(("first FF..FF special command", max(0, i - 10), min(n, i + 40)))
                break
    return out


def cut_candump(path, name, out_dir):
    with open(path, encoding="utf-8", errors="replace") as f:
        lines = [line.rstrip("\n") for line in f if "#" in line]
    keep, reasons = set(), []
    for reason, start, end in windows(lines, name):
        keep.update(range(start, end))
        reasons.append({"reason": reason, "lines": [start, end]})
    with open(os.path.join(out_dir, f"candump_{name}.log"), "w", encoding="utf-8") as f:
        for i in sorted(keep):
            f.write(lines[i] + "\n")
    ids = {}
    for line in lines:
        fid = frame_id(line)
        ids[fid] = ids.get(fid, 0) + 1
    return {"frames_in_recording": len(lines), "frames_kept": len(keep), "windows": reasons,
            "id_counts": dict(sorted(ids.items()))}


def to_plain(msg):
    from rosidl_runtime_py.convert import message_to_ordereddict
    return json.loads(json.dumps(message_to_ordereddict(msg), default=str))


def cut_bag(bag, out_dir):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = set(JSON_TOPICS) | set(ALL_TOPICS) | {"/joint_states", "/sil/base/odom", "/odom"}
    reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(t for t in wanted if t in types)))
    found = {}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        found.setdefault(topic, []).append((t_ns, data))
    messages, counts = {}, {}
    for topic, items in found.items():
        counts[topic] = len(items)
        kind = get_message(types[topic])
        if topic in ALL_TOPICS:
            picked = items
        else:
            step = max(1, len(items) // SAMPLES_PER_TOPIC)
            picked = items[::step][:SAMPLES_PER_TOPIC] + [items[-1]]
        rows = []
        for t_ns, data in picked:
            msg = deserialize_message(data, kind)
            value = msg.data if hasattr(msg, "data") and types[topic] == "std_msgs/msg/String" else to_plain(msg)
            if topic in JSON_TOPICS:
                try:
                    value = json.loads(value)
                except ValueError:
                    pass
            rows.append({"t": t_ns * 1e-9, "value": value})
        messages[topic] = {"type": types[topic], "count": counts[topic], "samples": rows}
    with open(os.path.join(out_dir, "messages.json"), "w", encoding="utf-8") as f:
        json.dump(messages, f, indent=1, sort_keys=True)
    return {"bag_topics": sorted(types), "kept_topics": counts}


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("logs")
    parser.add_argument("out")
    parser.add_argument("--label", required=True)
    args = parser.parse_args(argv)
    out_dir = os.path.join(args.out, args.label)
    os.makedirs(out_dir, exist_ok=True)
    selection = {"label": args.label, "buses": {}}
    for bus in BUSES:
        path = os.path.join(args.logs, f"record_candump_{bus}.log")
        if os.path.exists(path):
            selection["buses"][bus] = cut_candump(path, bus, out_dir)
        else:
            selection["buses"][bus] = "not recorded"
    bag = os.path.join(args.logs, "bag")
    selection["bag"] = cut_bag(bag, out_dir) if os.path.isdir(bag) else "no bag"
    for name in sorted(os.listdir(args.logs)):
        if name.endswith("_mission.json"):
            with open(os.path.join(args.logs, name), encoding="utf-8") as f:
                mission = json.load(f)
            selection["mission_file"] = name
            selection["mission_trace_samples"] = len(mission.get("trace") or [])
            mission["trace"] = f"{len(mission.get('trace') or [])} samples, not copied"
            with open(os.path.join(out_dir, "mission.json"), "w", encoding="utf-8") as f:
                json.dump(mission, f, indent=1, sort_keys=True)
    info = os.path.join(args.logs, "bag_info.txt")
    if os.path.exists(info):
        with open(info, encoding="utf-8") as src, open(os.path.join(out_dir, "bag_info.txt"), "w", encoding="utf-8") as dst:
            dst.write(src.read())
    with open(os.path.join(out_dir, "selection.json"), "w", encoding="utf-8") as f:
        json.dump(selection, f, indent=1, sort_keys=True)
    print(json.dumps({k: v for k, v in selection.items() if k != "bag"}, indent=1, sort_keys=True)[:4000])
    return 0


if __name__ == "__main__":
    sys.exit(main())
