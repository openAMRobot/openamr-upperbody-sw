# SPDX-License-Identifier: MIT
"""Acceptance gates of the strict Baseline 1 verdict beyond "8 of 8": component health for the whole run, Stop and
cancellation checks, contacts, and complete evidence. No ROS import: scripts/baseline1_strict_verdict.py uses it on the
artifacts of a leg, test/test_baseline1_strict.py on fixtures. The thresholds and lists are in
config/baseline1_gates.json, so a change to them is a reviewed change of one file.

Every gate returns a list of problems; an empty list is a pass of that gate. A missing record is a problem, never a pass.
Simulation evidence only: functional checks, never hardware, physical or safety evidence.
"""
import json
import os
import re

HEX40 = re.compile(r"[0-9a-f]{40}")
GATES_FILE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "baseline1_gates.json")


def load(path=None):
    with open(path or GATES_FILE, encoding="utf-8") as f:
        return json.load(f)


def _checks(phase_record):
    """name -> (ok, value) of the checks (quantities with a limit) of one phase record."""
    return {q["name"]: (q.get("ok"), q.get("value")) for q in phase_record.get("quantities", []) if q.get("ok") is not None}


def _phase(mission, name):
    return next((r for r in (mission or {}).get("phases", []) if r.get("phase") == name), None)


# ------------------------------------------------------------------ mission result
def mission_result(mission):
    """The mission as a whole passed: result PASS and no failed check outside the eight phase records (for example the
    component liveness checks rig.run adds after the mission function returns)."""
    if mission is None:
        return ["no mission log"]
    problems = []
    if mission.get("result") != "PASS":
        problems.append(f"mission result {mission.get('result')!r}: {mission.get('reason') or 'no reason'}")
    in_phases = {f["text"] for r in mission.get("phases", []) for f in r.get("failures", [])}
    outside = [text for text in mission.get("failures", []) if text not in in_phases]
    if outside:
        problems.append("failed checks outside the phases: " + "; ".join(outside[:5]))
    return problems


# ------------------------------------------------------------------ component health for the whole run
def liveness(mission, config):
    """Each required component's own work counter (written to every trace sample by baseline1.Trace) advances from the
    first to the last sample of the mission, with no gap longer than max_gap_sim_s of Gazebo time. A counter that is
    never recorded is a problem: the run does not show the component was alive."""
    if mission is None:
        return ["no mission log"]
    gate = config["liveness"]
    trace = [s for s in mission.get("trace", []) if s.get("sim") is not None]
    if len(trace) < 2:
        return ["liveness: no trace samples"]
    problems = []
    for name, key in sorted(gate["counters"].items()):
        points = [(s["sim"], s.get(key)) for s in trace]
        if all(value is None for _t, value in points):
            problems.append(f"liveness {name}: counter {key} not recorded in the trace")
            continue
        missing = sum(1 for _t, value in points if value is None)
        if missing:
            problems.append(f"liveness {name}: counter {key} missing in {missing} of {len(points)} samples")
        known = [(t, v) for t, v in points if v is not None]
        last_change, last_value, worst = known[0][0], known[0][1], 0.0
        for t, value in known[1:]:
            if value != last_value:
                worst = max(worst, t - last_change)
                last_change, last_value = t, value
        worst = max(worst, trace[-1]["sim"] - last_change)
        if worst > gate["max_gap_sim_s"]:
            problems.append(f"liveness {name}: counter {key} did not advance for {worst:.1f} s of Gazebo time "
                            f"(limit {gate['max_gap_sim_s']} s)")
        if known[0][0] - trace[0]["sim"] > gate["max_gap_sim_s"]:
            problems.append(f"liveness {name}: counter {key} first recorded {known[0][0] - trace[0]['sim']:.1f} s after "
                            "the mission started")
    for name in gate.get("end_of_run_checks", []):
        found = [q for q in mission.get("quantities", []) if q.get("name") == name]
        if found and found[-1].get("value") is not True:
            problems.append(f"liveness {name}: {found[-1].get('value')!r} at the end of the run")
    return problems


# ------------------------------------------------------------------ Stop and cancellation
def stop_cancel(mission, config):
    """The Stop checks named in the config ran in their phase and passed (a check that did not run is a problem, so an
    early return cannot pass the phase on fewer checks), and every required cancellation check ran and passed."""
    if mission is None:
        return ["no mission log"]
    gate = config["stop_cancel"]
    problems = []
    for phase, names in sorted(gate["required_checks"].items()):
        record = _phase(mission, phase)
        checks = _checks(record or {})
        for name in names:
            if name not in checks:
                problems.append(f"{phase}: check {name} did not run")
            elif checks[name][0] is not True:
                problems.append(f"{phase}: check {name} failed ({checks[name][1]!r})")
    cancel = gate["cancellation"]
    all_checks = {}
    for record in mission.get("phases", []):
        all_checks.update(_checks(record))
    found = sorted(name for name in all_checks if re.match(cancel["check_pattern"], name))
    if len(found) < cancel["min_checks"]:
        problems.append(f"cancellation: {len(found)} cancellation checks ran ({cancel['check_pattern']}), "
                        f"at least {cancel['min_checks']} required: {cancel['note']}")
    problems += [f"cancellation: check {name} failed ({all_checks[name][1]!r})" for name in found
                 if all_checks[name][0] is not True]
    return problems


# ------------------------------------------------------------------ contacts
def contact_allowed(event, allowed):
    """The first allowed-list entry that accounts for this contact event, or None. An entry with a `window` accounts
    only for events inside that named window (config contacts.windows, from the mission's time marks)."""
    a, b = event["a"], event["b"]
    for entry in allowed:
        if event["phase"] not in entry["phases"]:
            continue
        if entry.get("window") is not None and event.get("window") != entry["window"]:
            continue
        for x, y in ((a, b), (b, a)):
            if re.search(entry["a"], x) and re.search(entry["b"], y):
                if entry.get("max_force_n") is None or (event.get("max_force_n") or 0.0) <= entry["max_force_n"]:
                    return entry
    return None


def contacts(mission, config):
    """Contact sensors were on and delivered messages, and every recorded contact event is in the allowed list with
    its reason (within its force limit); any other contact is a problem."""
    if mission is None:
        return ["no mission log"]
    gate = config["contacts"]
    record = mission.get("contacts")
    if not record:
        return ["contacts: not recorded (contact sensors off or no recorder in the mission)"]
    problems = []
    if not record.get("sensors_on"):
        problems.append("contacts: contact sensors were not on in this run")
    if len(record.get("sensor_links") or []) < gate["min_sensor_links"]:
        problems.append(f"contacts: {len(record.get('sensor_links') or [])} sensor links, "
                        f"at least {gate['min_sensor_links']} required")
    if sum((record.get("messages") or {}).values()) < gate["min_messages"]:
        problems.append(f"contacts: {sum((record.get('messages') or {}).values())} contact messages received, "
                        f"at least {gate['min_messages']} required (the sensors may not publish)")
    unaccounted = [e for e in record.get("events", []) if contact_allowed(e, gate["allowed"]) is None]
    for event in unaccounted[:10]:
        where = event["phase"] + (f" ({event['window']})" if event.get("window") else "")
        problems.append(f"contacts: unaccounted contact in {where}: {event['a']} with {event['b']} "
                        f"({event.get('count')} samples, max {event.get('max_force_n')} N)")
    if len(unaccounted) > 10:
        problems.append(f"contacts: {len(unaccounted) - 10} more unaccounted contact events")
    return problems


def windows(marks, definitions):
    """Named Gazebo-time windows from the mission's time marks [(label, t)] (scenarios._mark) and the definitions
    {name: [start label, end label]} of config contacts.windows: the first start mark, then the first end mark at or
    after it. A window whose marks are missing is absent, so an entry that needs it accounts for nothing."""
    found = {}
    for name, labels in sorted(definitions.items()):
        if name.startswith("_"):
            continue
        start_label, end_label = labels
        start = next((t for label, t in marks if label == start_label), None)
        if start is None:
            continue
        end = next((t for label, t in marks if label == end_label and t >= start), None)
        if end is not None:
            found[name] = [start, end]
    return found


def window_of(found):
    """-> function(t) naming the window of `found` (windows()) that contains Gazebo time t, else None."""
    spans = sorted(found.items(), key=lambda item: item[1][0])
    return lambda t: next((name for name, (start, end) in spans if start <= t <= end), None)


def contact_events(rows, phase_of, window_of=None):
    """Rows {t, a, b, fn} -> events, one per (phase, window, a, b): first and last Gazebo time, sample count, largest
    normal force. `phase_of(t)` names the phase running at Gazebo time t (None before and after the mission);
    `window_of(t)` the named contact window (None outside every window, and when no function is given)."""
    events = {}
    for row in rows:
        phase = phase_of(row["t"]) or "outside"
        window = window_of(row["t"]) if window_of is not None else None
        key = (phase, window, row["a"], row["b"])
        event = events.setdefault(key, {"phase": phase, "window": window, "a": row["a"], "b": row["b"],
                                        "first_t": row["t"], "last_t": row["t"], "count": 0, "max_force_n": 0.0})
        event["count"] += 1
        event["last_t"] = max(event["last_t"], row["t"])
        fn = row.get("fn")
        if fn is not None and fn == fn:          # NaN when the sensor gave no normal
            event["max_force_n"] = round(max(event["max_force_n"], abs(fn)), 3)
    return sorted(events.values(), key=lambda e: (e["first_t"], e["a"], e["b"]))


# ------------------------------------------------------------------ evidence
def evidence(leg_dir, declaration, mission, config):
    """The artifacts of one leg are complete: bag with metadata, one non-empty candump per virtual bus, the mission
    JSON with every phase timed, run_info, pinned commits as full hashes, seed and configuration declared."""
    gate = config["evidence"]
    problems = []
    logs = next((os.path.join(leg_dir, rel) for rel in gate["log_dirs"] if os.path.isdir(os.path.join(leg_dir, rel))), None)
    if logs is None:
        return ["evidence: no build1_logs directory"]
    if not os.path.isfile(os.path.join(logs, "bag", "metadata.yaml")):
        problems.append("evidence: no ROS bag (bag/metadata.yaml)")
    for bus in gate["can_buses"]:
        path = os.path.join(logs, f"record_candump_{bus}.log")
        if not os.path.isfile(path):
            problems.append(f"evidence: no candump of {bus}")
            continue
        with open(path, encoding="utf-8", errors="replace") as f:
            frames = sum(1 for line in f if "#" in line)
        if frames < gate["min_frames_per_bus"]:
            problems.append(f"evidence: candump of {bus} has {frames} frames")
    declaration = declaration or {}
    pins = declaration.get("pins") or {}
    for name in gate["pinned_repositories"]:
        if not isinstance(pins.get(name), str) or not HEX40.fullmatch(pins[name]):
            problems.append(f"evidence: pinned commit of {name} missing or not a full hash ({pins.get(name)!r})")
    for key in gate["declared_settings"]:
        if declaration.get(key) in (None, ""):
            problems.append(f"evidence: setting {key} not declared")
    if mission is None:
        return problems + ["evidence: no mission JSON"]
    seed = (mission.get("baseline1") or {}).get("seed")
    if seed is None or str(seed) != str(declaration.get("seed")):
        problems.append(f"evidence: mission seed {seed!r} differs from the declared seed {declaration.get('seed')!r}")
    if not mission.get("run"):
        problems.append("evidence: no run_info in the mission JSON")
    records = {r.get("phase"): r for r in mission.get("phases", [])}
    for name in gate["phases"]:
        record = records.get(name)
        if record is None or record.get("sim_start") is None or record.get("sim_end") is None:
            problems.append(f"evidence: phase {name} has no Gazebo start and end time")
    if len(mission.get("trace", [])) < gate["min_trace_samples"]:
        problems.append(f"evidence: {len(mission.get('trace', []))} trace samples, at least {gate['min_trace_samples']}")
    return problems


def all_gates(leg_dir, declaration, mission, config):
    return (mission_result(mission) + liveness(mission, config) + stop_cancel(mission, config)
            + contacts(mission, config) + evidence(leg_dir, declaration, mission, config))
