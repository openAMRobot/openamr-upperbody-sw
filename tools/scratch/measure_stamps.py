# SPDX-License-Identifier: MIT
"""Scratch (not for merge): measure per-stream timestamps of a recorded bag.

For each topic: header stamps and bag receipt times, equal and decreasing consecutive values, groups of equal stamps,
whether an equal-stamp pair carries different content, delta histograms, and how the stamps relate to /clock.
Read only. Simulation evidence only.
"""
import argparse
import bisect
import collections
import json
import statistics
from pathlib import Path

from rosbags.highlevel import AnyReader


def stamp_ns(msg):
    h = getattr(msg, "header", None)
    if h is None:
        return None
    return int(h.stamp.sec) * 1_000_000_000 + int(h.stamp.nanosec)


def content(msg):
    for f in ("position", "velocity", "effort"):
        if hasattr(msg, f):
            return tuple(round(float(v), 12) for f2 in ("position", "velocity", "effort") for v in getattr(msg, f2, []))
    lin = getattr(msg, "linear", None)
    if lin is not None:
        ang = msg.angular
        return (lin.x, lin.y, lin.z, ang.x, ang.y, ang.z)
    return None


def series_stats(values):
    eq = dec = 0
    max_dec = 0
    groups = collections.Counter()
    run = 1
    for a, b in zip(values, values[1:]):
        if b == a:
            eq += 1
            run += 1
        else:
            if run > 1:
                groups[run] += 1
            run = 1
            if b < a:
                dec += 1
                max_dec = max(max_dec, a - b)
    if run > 1:
        groups[run] += 1
    deltas = collections.Counter(b - a for a, b in zip(values, values[1:]))
    span = (values[-1] - values[0]) if len(values) > 1 else 0
    return {
        "n": len(values),
        "equal_consecutive": eq,
        "decreasing_consecutive": dec,
        "max_decrease_ns": max_dec,
        "equal_group_sizes": dict(sorted(groups.items())),
        "span_s": span / 1e9,
        "rate_hz_of_this_time": (len(values) - 1) / (span / 1e9) if span > 0 else None,
        "top_deltas_ns": deltas.most_common(8),
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("bag", type=Path)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--topics", nargs="*", default=None)
    a = p.parse_args()
    meta = (a.bag / "metadata.yaml").read_text()
    data = collections.defaultdict(list)  # topic -> [(receipt, header, content)]
    clock = []  # (receipt, clock value)
    skipped = set()
    with AnyReader([a.bag]) as r:
        types = {c.topic: c.msgtype for c in r.connections}
        conns = [c for c in r.connections if a.topics is None or c.topic in a.topics or c.topic == "/clock"]
        pubs = collections.Counter(c.topic for c in r.connections)
        for conn, t, raw in r.messages(connections=conns):
            if "Image" in conn.msgtype and "CameraInfo" not in conn.msgtype:
                # Images: header only (deserializing is slow but needed for the stamp)
                pass
            try:
                msg = r.deserialize(raw, conn.msgtype)
            except KeyError:
                skipped.add(conn.topic)
                continue
            if conn.topic == "/clock":
                clock.append((t, int(msg.clock.sec) * 1_000_000_000 + int(msg.clock.nanosec)))
                continue
            data[conn.topic].append((t, stamp_ns(msg), content(msg)))
    report = {"skipped_unknown_type": sorted(skipped), "bag": str(a.bag), "connections_per_topic": dict(pubs), "types": types, "topics": {}}
    if clock:
        cv = [c for _, c in clock]
        report["clock"] = {"value": series_stats(cv), "receipt": series_stats([t for t, _ in clock]),
                           "receipt_minus_value_ns": [min(t - c for t, c in clock), statistics.median(t - c for t, c in clock),
                                                      max(t - c for t, c in clock)]}
        clock_values = sorted(set(cv))
    for topic, rows in sorted(data.items()):
        rec = [t for t, _, _ in rows]
        entry = {"type": types[topic], "receipt": series_stats(rec)}
        hdr = [h for _, h, _ in rows]
        if all(h is not None for h in hdr):
            entry["header"] = series_stats(hdr)
            lags = [t - h for t, h, _ in rows]
            entry["receipt_minus_header_ns"] = [min(lags), statistics.median(lags), max(lags)]
            same = diff = 0
            examples = []
            for i in range(1, len(rows)):
                if hdr[i] == hdr[i - 1]:
                    if rows[i][2] == rows[i - 1][2]:
                        same += 1
                    else:
                        diff += 1
                    if len(examples) < 6:
                        examples.append({"index": i, "stamp_ns": hdr[i], "receipt_prev": rec[i - 1], "receipt": rec[i],
                                         "content_equal": rows[i][2] == rows[i - 1][2],
                                         "prev_delta_ns": hdr[i - 1] - hdr[i - 2] if i >= 2 else None,
                                         "next_delta_ns": hdr[i + 1] - hdr[i] if i + 1 < len(hdr) else None})
            entry["equal_header_pairs"] = {"content_identical": same, "content_different": diff, "examples": examples}
            if clock:
                on_clock = sum(1 for h in hdr if clock_values[min(bisect.bisect_left(clock_values, h), len(clock_values) - 1)] == h)
                entry["header_stamps_equal_to_a_clock_value"] = on_clock
        else:
            same = sum(1 for i in range(1, len(rows)) if rec[i] == rec[i - 1] and rows[i][2] == rows[i - 1][2])
            entry["equal_receipt_pairs_content_identical"] = same
        if clock:
            # receipt times that are not a /clock value would mean the recorder's clock is not /clock
            on = sum(1 for t in rec if clock_values[min(bisect.bisect_left(clock_values, t), len(clock_values) - 1)] == t)
            entry["receipt_equal_to_a_clock_value"] = on
        report["topics"][topic] = entry
    import yaml
    info = yaml.safe_load(meta)["rosbag2_bagfile_information"]
    qos = {}
    for t in info["topics_with_message_count"]:
        md = t["topic_metadata"]
        prof = md.get("offered_qos_profiles", "")
        prof = yaml.safe_load(prof) if isinstance(prof, str) and prof else prof
        qos[md["name"]] = len(prof) if isinstance(prof, list) else prof
    report["publishers_per_topic_from_offered_qos"] = qos
    a.out.write_text(json.dumps(report, indent=1, default=str))
    for topic in ("/clock",):
        if clock:
            print("/clock", json.dumps(report["clock"]))
    for topic, e in report["topics"].items():
        h = e.get("header", {})
        print(f"{topic}: n={e['receipt']['n']} receipt_eq={e['receipt']['equal_consecutive']} receipt_dec={e['receipt']['decreasing_consecutive']}"
              f" header_eq={h.get('equal_consecutive')} header_dec={h.get('decreasing_consecutive')}"
              f" hdr_rate={h.get('rate_hz_of_this_time')} pairs={e.get('equal_header_pairs', {}).get('content_identical')}/"
              f"{e.get('equal_header_pairs', {}).get('content_different')}")


if __name__ == "__main__":
    main()
