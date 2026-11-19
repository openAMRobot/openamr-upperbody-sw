# SPDX-License-Identifier: MIT
"""Scratch (not for merge): choose and cut the 5 s test fixture of run 44448676604.

For each candidate receipt-time window the converter's own read_bag and align run on the recorded
messages of that window (Proposed profile, rules enabled by name). The first window that aligns, holds
a lift command, an equal header stamp on /arm/joint_states and a changing action vector is cut with
embodied-ai python/tools/cut_bag_slice.py, bytes unchanged; the cut is read and aligned again.
Simulation evidence only.
"""
import argparse
import base64
import io
import json
import subprocess
import sys
import tarfile
from pathlib import Path

ENABLED = ("proposed_capture_profile", "episode_origin", "controller_reference", "command_semantics",
           "vector_segments", "stop_index", "range_tolerance", "held_command",
           "lift_positioning_from_command", "lift_phase_report")


def window(data, start, end):
    return {k: [s for s in v if start <= s.receipt_ns < end] for k, v in data.items()}


def equal_stamps(samples):
    return sum(1 for a, b in zip(samples, samples[1:]) if a.time_ns == b.time_ns)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--bag", type=Path, required=True)
    p.add_argument("--profile", type=Path, required=True)
    p.add_argument("--tool", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args()
    from oa_embodied_ai.recording.bag import align, read_bag
    from oa_embodied_ai.recording.profile import validate_profile

    profile = validate_profile(json.loads(a.profile.read_text()), enabled=ENABLED)
    # equal stamps are counted on the raw messages, so read once without keep_last folding
    raw_profile = json.loads(json.dumps(profile))
    data = read_bag(a.bag, profile)
    topics = sorted({s["topic"] for s in profile["streams"]})
    from rosbags.highlevel import AnyReader
    with AnyReader([a.bag]) as reader:
        info = sorted(c.topic for c in reader.connections if c.topic.endswith("/camera_info"))
        raw = {s["id"]: [] for s in profile["streams"] if s["id"] == "arm_joint_states"}
        conns = [c for c in reader.connections if c.topic == "/arm/joint_states"]
        for conn, receipt, payload in reader.messages(connections=conns):
            msg = reader.deserialize(payload, conn.msgtype)
            raw["arm_joint_states"].append((receipt, msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec))
    del raw_profile
    candidates = []
    for start_s in (37.5, 37.7, 37.0, 36.5):  # last /cmd_vel before the lift at 37.82 s; lift command at 40.524 s
        start, end = int(start_s * 1e9), int((start_s + 5.0) * 1e9)
        sub = window(data, start, end)
        stamps = [t for r, t in raw["arm_joint_states"] if start <= r < end]
        result = {"start_s": start_s, "end_s": start_s + 5.0,
                  "counts": {k: len(v) for k, v in sub.items()},
                  "arm_joint_states_equal_header_stamps": sum(1 for x, y in zip(stamps, stamps[1:]) if x == y),
                  "cmd_vel_equal_receipt": sum(1 for x, y in zip(sub["cmd_vel"], sub["cmd_vel"][1:]) if x.receipt_ns == y.receipt_ns),
                  "lift_commands": [(s.receipt_ns, int(s.message.data)) for s in sub["sil_lift_command"]]}
        try:
            report = {}
            frames = align(sub, profile, report=report)
            actions = [tuple(f["action"]) for f in frames]
            result.update(status="ALIGNED", frames=len(frames), distinct_actions=len(set(actions)),
                          changes=sum(1 for x, y in zip(actions, actions[1:]) if x != y),
                          origin=report.get("episode_origin"),
                          lift_state=[min(f["state"][16] for f in frames), max(f["state"][16] for f in frames)],
                          lift_action=sorted({f["action"][16] for f in frames}))
        except Exception as exc:  # report each refusal; never worked around
            result.update(status="REFUSED", error=f"{type(exc).__name__}: {exc}")
        candidates.append(result)
        print(json.dumps(result), flush=True)
    chosen = next((c for c in candidates if c["status"] == "ALIGNED" and c["lift_commands"]
                   and c["arm_joint_states_equal_header_stamps"] > 0 and c["changes"] > 0), None)
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / "candidates.json").write_text(json.dumps(candidates, indent=1))
    if chosen is None:
        print("NO WINDOW MEETS THE CRITERIA")
        return 1
    print("CHOSEN", json.dumps(chosen))
    bag_out = a.out / "baseline1_slice"
    start, end = int(chosen["start_s"] * 1e9), int(chosen["end_s"] * 1e9)
    cmd = [sys.executable, str(a.tool), str(a.bag), str(bag_out), "--start-ns", str(start), "--end-ns", str(end),
           "--report", str(a.out / "slice_report.json")] + [x for t in topics + info for x in ("--topic", t)]
    print(" ".join(cmd))
    subprocess.run(cmd, check=True)
    sliced = read_bag(bag_out, profile)
    report = {}
    frames = align(sliced, profile, report=report)
    print("SLICE ALIGNED", len(frames), json.dumps(report.get("episode_origin")))
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w") as tar:
        for path in sorted(bag_out.rglob("*")):
            info_ = tar.gettarinfo(str(path), arcname=path.relative_to(a.out).as_posix())
            info_.mtime, info_.uid, info_.gid, info_.uname, info_.gname = 0, 0, 0, "", ""
            with open(path, "rb") as fh:
                tar.addfile(info_, fh)
        rep = a.out / "slice_report.json"
        info_ = tar.gettarinfo(str(rep), arcname="slice_report.json")
        info_.mtime, info_.uid, info_.gid, info_.uname, info_.gname = 0, 0, 0, "", ""
        with open(rep, "rb") as fh:
            tar.addfile(info_, fh)
    blob = buf.getvalue()
    import hashlib
    print("TAR_SHA256", hashlib.sha256(blob).hexdigest(), "BYTES", len(blob))
    text = base64.b64encode(blob).decode()
    print("BEGIN_FIXTURE_B64")
    for i in range(0, len(text), 4000):
        print("B64:" + text[i:i + 4000])
    print("END_FIXTURE_B64")
    return 0


if __name__ == "__main__":
    sys.exit(main())
