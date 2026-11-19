# SPDX-License-Identifier: MIT
"""Offline mission report: a recorded bag plus the mission JSON (and optional candump logs) to one static HTML page.

No ROS import here. bag.py reads a bag into (topic, message, receive time ns) tuples; this module takes any iterable
of those, with messages that have the ROS field names, so the unit tests feed it plain objects.

Plots: station error, localization error, joint states, lift position, real-time factor, CAN frame rate, contact
force, and the phase timeline from the mission JSON, with the phase table. Every plot is on the Gazebo time base
except the CAN rate (candump wall time). The page is a record of a simulation run: never hardware, physical
acceptance or safety evidence, and it decides nothing; the mission JSON holds the verdicts.
"""
import html
import json
import math
import re

from openamrobot_control_room.state import angle_diff, yaw_of

# Fixed categorical order (blue, orange, aqua, yellow, magenta, green, violet, red), light and dark steps.
SERIES_LIGHT = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948")
SERIES_DARK = ("#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#0ca30c", "#9085e9", "#e66767")
MAX_POINTS = 1200
ARM_JOINTS = tuple(f"openarm_{side}_joint{i}" for side in ("left", "right") for i in range(1, 8))
VERDICT_TEXT = {"PASS": "pass", "FAIL": "fail", "BLOCKED": "blocked", "NOT RUN": "not run"}


def _stamp(msg):
    header = getattr(msg, "header", None)
    sec = getattr(getattr(header, "stamp", None), "sec", None)
    if not isinstance(sec, int):
        return None
    t = sec + header.stamp.nanosec * 1e-9
    return t if t > 0 else None


def _pose2d(pose):
    q = pose.orientation
    return pose.position.x, pose.position.y, yaw_of(q.x, q.y, q.z, q.w)


def extract(messages, station=None):
    """Series from bag messages. `station`: (x, y, yaw) for the station error, or None."""
    out = {"station_error": [], "station_along": [], "station_left": [], "loc_error": [], "loc_yaw_error": [],
           "lift_js": [], "lift_physics": [], "rtf": [], "contacts": {}, "joints": {j: [] for j in ARM_JOINTS},
           "item_z": [], "topics": {}, "sim_span": None, "run_info": None, "mission_status": []}
    clock = None
    truth = None
    first_sim = last_sim = None
    for topic, msg, t_ns in messages:
        out["topics"][topic] = out["topics"].get(topic, 0) + 1
        if topic == "/clock":
            clock = msg.clock.sec + msg.clock.nanosec * 1e-9
            first_sim = clock if first_sim is None else first_sim
            last_sim = clock
            continue
        t = _stamp(msg) or clock
        if t is None:
            continue
        if topic == "/build1/ground_truth/odom":
            truth = _pose2d(msg.pose.pose)
            if station is not None:
                dx, dy = truth[0] - station[0], truth[1] - station[1]
                c, s = math.cos(station[2]), math.sin(station[2])
                out["station_error"].append((t, math.hypot(dx, dy)))
                out["station_along"].append((t, dx * c + dy * s))
                out["station_left"].append((t, -dx * s + dy * c))
        elif topic == "/amcl_pose" and truth is not None:
            est = _pose2d(msg.pose.pose)
            out["loc_error"].append((t, math.hypot(est[0] - truth[0], est[1] - truth[1])))
            out["loc_yaw_error"].append((t, abs(angle_diff(est[2], truth[2]))))
        elif topic in ("/joint_states", "/sil/gz/joint_states"):
            names = list(msg.name)
            positions = list(msg.position)
            if "lift_joint" in names and len(positions) > names.index("lift_joint"):
                key = "lift_js" if topic == "/joint_states" else "lift_physics"
                out[key].append((t, positions[names.index("lift_joint")]))
            if topic == "/joint_states":
                for i, name in enumerate(names):
                    if name in out["joints"] and i < len(positions):
                        out["joints"][name].append((t, positions[i]))
        elif topic == "/build1/rtf_stats":
            try:
                value = json.loads(msg.data).get("rtf_last")
            except (ValueError, AttributeError):
                value = None
            if value is not None:
                out["rtf"].append((t, float(value)))
        elif topic == "/build1/ground_truth/item/pose":
            out["item_z"].append((t, msg.pose.position.z))
        elif topic == "/build1/run_info" and out["run_info"] is None:
            try:
                out["run_info"] = json.loads(msg.data)
            except (ValueError, AttributeError):
                pass
        elif topic in ("/baseline1/mission_status", "/grocery_demo/phase"):
            out["mission_status"].append((t, topic, getattr(msg, "data", "")[:300]))
        elif topic.startswith("/build1/contacts/") or topic == "/control_room/contacts/max_force":
            if hasattr(msg, "contacts"):
                force = 0.0
                for contact in msg.contacts:
                    total = [0.0, 0.0, 0.0]
                    for wrench in getattr(contact, "wrenches", []):
                        f = wrench.body_1_wrench.force
                        total = [total[0] + f.x, total[1] + f.y, total[2] + f.z]
                    force = max(force, math.sqrt(sum(v * v for v in total)))
            else:
                force = float(msg.data)
            out["contacts"].setdefault(topic.rsplit("/", 1)[-1], []).append((t, force))
    out["sim_span"] = (first_sim, last_sim) if first_sim is not None else None
    return out


_CANDUMP = re.compile(r"^\((?P<t>[0-9.]+)\)\s+(?P<bus>\S+)\s+(?P<id>[0-9A-Fa-f]+)#")


def candump_rates(lines_by_bus, bin_s=1.0):
    """candump -L lines per bus to (seconds since the first frame of any bus, frames per second) per bus."""
    times = {}
    for bus, lines in lines_by_bus.items():
        stamps = []
        for line in lines:
            match = _CANDUMP.match(line.strip())
            if match:
                stamps.append(float(match.group("t")))
        times[bus] = stamps
    start = min((s[0] for s in times.values() if s), default=None)
    rates = {}
    for bus, stamps in times.items():
        bins = {}
        for t in stamps:
            k = int((t - start) // bin_s)
            bins[k] = bins.get(k, 0) + 1
        if bins:
            rates[bus] = [((k + 0.5) * bin_s, bins.get(k, 0) / bin_s) for k in range(0, max(bins) + 1)]
        else:
            rates[bus] = []
    return rates


def candump_decoded(lines_by_bus, arm_every=25):
    """Engineering series from candump -L lines (can_decode.py), on candump's host wall time since the first frame of
    any bus. Arm buses: every `arm_every`th frame of each ID (they carry about 2000 frames per simulated second).
    Returns {"series": {name: [(t, value)]}, "commands": [lift command rows], "raw": [first frame of each bus and ID]}."""
    from openamrobot_control_room import can_decode as cd
    parsed = {bus: [p for p in (cd.parse_candump(line) for line in lines) if p] for bus, lines in lines_by_bus.items()}
    start = min((p[0][0] for p in parsed.values() if p), default=None)
    series, commands, raw, seen, counter = {}, [], [], set(), {}

    def add(name, t, value):
        if value is not None:
            series.setdefault(name, []).append((t - start, value))
    for bus, frames in sorted(parsed.items()):
        for t, _bus, can_id, data, ext, _fd in frames:
            if bus in cd.ARM_SIDE:
                counter[(bus, can_id)] = counter.get((bus, can_id), 0) + 1
                if (counter[(bus, can_id)] - 1) % arm_every:
                    continue
            r = cd.decode(bus, can_id, data, ext)
            if (bus, can_id) not in seen:
                seen.add((bus, can_id))
                raw.append({"bus": bus, "id": f"{can_id:X}", "data": data.hex().upper(), "t": round(t - start, 3),
                            "decoded": {k: v for k, v in r.items() if k not in ("device",)}})
            kind = r.get("kind")
            if bus == "vcan_drive" and kind in ("SDO write", "SDO read reply") and r.get("index") in ("60FFh", "606Ch"):
                what = "target (60FFh write)" if r["index"] == "60FFh" else "actual (606Ch read)"
                add(f"drive {r['wheel']} {what}", t, r["wheel_m_s"])
            elif bus == "vcan_aux" and kind == "TPDO1 status":
                add("lift TPDO1 position", t, r["position_mm"])
                add("lift TPDO1 state", t, cd.LIFT_STATES.index(r["state"]) if r["state"] in cd.LIFT_STATES else None)
            elif bus == "vcan_aux" and kind == "TPDO2 inputs and drive":
                add("lift TPDO2 velocity", t, r["velocity_mm_s"])
            elif bus == "vcan_aux" and kind == "RPDO1 command" and (not commands or commands[-1]["seq"] != r["seq"]):
                commands.append({"t": round(t - start, 3), "seq": r["seq"], "op": r["op"], "stop": r["stop"]})
            elif bus == "vcan_bms" and r.get("data_id") == "90h" and "soc_percent" in r:
                add("BMS SOC", t, r["soc_percent"])
                add("BMS pack voltage", t, r["pack_voltage_v"])
            elif bus in cd.ARM_SIDE and kind in ("MIT command", "state feedback") and r["motor"] <= 7:
                add(f"{r['joint']} {'command' if kind == 'MIT command' else 'feedback'}", t, r["q_rad"])
    return {"series": series, "commands": commands, "raw": raw}


def _thin(points, limit=MAX_POINTS):
    if len(points) <= limit:
        return points
    step = len(points) / limit
    return [points[int(i * step)] for i in range(limit)] + [points[-1]]


def _ticks(low, high, count=5):
    if high <= low:
        high = low + 1.0
    raw = (high - low) / count
    mag = 10 ** math.floor(math.log10(raw))
    step = next(m * mag for m in (1, 2, 2.5, 5, 10) if m * mag >= raw)
    first = math.ceil(low / step) * step
    return [first + i * step for i in range(int((high - first) / step) + 1)]


def _fmt(v):
    if v is None:
        return "-"
    if isinstance(v, float):
        if v == 0:
            return "0"
        return f"{v:.4g}" if abs(v) < 1e4 else f"{v:.0f}"
    return str(v)


def svg_chart(chart_id, series, x_label, y_label, bands=(), width=880, height=240):
    """A line chart as inline SVG. `series`: [(name, [(x, y)])]. `bands`: [(x0, x1, label)] phase backdrop."""
    series = [(name, _thin([(x, y) for x, y in pts if x is not None and y is not None and math.isfinite(y)]))
              for name, pts in series]
    series = [(n, p) for n, p in series if p]
    if not series:
        return f'<p class="empty" id="{chart_id}">No data for this plot in the recording.</p>'
    left, right, top, bottom = 64, 16, 12, 36
    xs = [x for _, p in series for x, _ in p] + [b for band in bands for b in band[:2] if b is not None]
    ys = [y for _, p in series for _, y in p]
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    if y1 - y0 < 1e-4:    # a flat line (or float noise on one): a readable span around it
        mid = (y0 + y1) / 2.0
        half = max(0.5e-4, 0.05 * abs(mid))
        y0, y1 = mid - half, mid + half
    pad = 0.05 * (y1 - y0)
    y0, y1 = y0 - pad, y1 + pad
    x1 = x1 if x1 > x0 else x0 + 1.0
    w, h = width - left - right, height - top - bottom

    def sx(x):
        return left + (x - x0) / (x1 - x0) * w

    def sy(y):
        return top + (1 - (y - y0) / (y1 - y0)) * h
    parts = [f'<svg id="{chart_id}" class="chart" viewBox="0 0 {width} {height}" role="img" '
             f'aria-label="{html.escape(y_label)} against {html.escape(x_label)}">']
    for i, (b0, b1, label) in enumerate(bands):
        if b0 is None or b1 is None:
            continue
        parts.append(f'<rect class="band{i % 2}" x="{sx(b0):.1f}" y="{top}" width="{max(0.5, sx(b1) - sx(b0)):.1f}" '
                     f'height="{h}"><title>{html.escape(label)}</title></rect>')
    yticks = _ticks(y0, y1)
    step = yticks[1] - yticks[0] if len(yticks) > 1 else 1.0
    decimals = min(6, max(0, -math.floor(math.log10(step)) + 1)) if step < 1 else 0
    for t in yticks:
        parts.append(f'<line class="grid" x1="{left}" x2="{left + w}" y1="{sy(t):.1f}" y2="{sy(t):.1f}"/>'
                     f'<text class="tick" x="{left - 6}" y="{sy(t) + 4:.1f}" text-anchor="end">{t:.{decimals}f}</text>')
    for t in _ticks(x0, x1, 8):
        parts.append(f'<text class="tick" x="{sx(t):.1f}" y="{top + h + 16}" text-anchor="middle">{_fmt(round(t, 3))}</text>')
    parts.append(f'<line class="axis" x1="{left}" x2="{left + w}" y1="{top + h}" y2="{top + h}"/>')
    parts.append(f'<text class="label" x="{left + w / 2:.1f}" y="{height - 4}" text-anchor="middle">{html.escape(x_label)}</text>')
    parts.append(f'<text class="label" x="12" y="{top + h / 2:.1f}" transform="rotate(-90 12 {top + h / 2:.1f})" '
                 f'text-anchor="middle">{html.escape(y_label)}</text>')
    data = []
    for i, (name, pts) in enumerate(series):
        path = " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in pts)
        parts.append(f'<polyline class="line s{i % 8}" points="{path}"><title>{html.escape(name)}</title></polyline>')
        data.append({"name": name, "slot": i % 8, "pts": [[round(x, 3), round(y, 5)] for x, y in pts]})
    parts.append(f'<line class="cross" x1="0" x2="0" y1="{top}" y2="{top + h}" visibility="hidden"/>')
    parts.append(f'<rect class="hit" x="{left}" y="{top}" width="{w}" height="{h}" data-x0="{x0}" data-x1="{x1}" '
                 f'data-left="{left}" data-w="{w}"/></svg>')
    legend = ""
    if len(series) > 1:
        legend = '<div class="legend">' + "".join(
            f'<span><i class="key s{i % 8}"></i>{html.escape(n)}</span>' for i, (n, _) in enumerate(series)) + "</div>"
    payload = html.escape(json.dumps({"x": x_label, "y": y_label, "series": data}), quote=True)
    return f'<div class="plot" data-series="{payload}">{legend}{"".join(parts)}<div class="tip" hidden></div></div>'


def phase_bands(mission):
    return [(r.get("sim_start"), r.get("sim_end"), r["phase"]) for r in (mission or {}).get("phases", []) if r.get("ran")]


def _verdict(record):
    if record.get("exception"):
        return "BLOCKED" if record["exception"].get("type") == "Blocked" else "FAIL"
    if not record.get("ran"):
        return "NOT RUN"
    if record.get("failures"):
        return "FAIL"
    return "PASS" if any(q.get("ok") is not None for q in record.get("quantities", [])) else "FAIL"


def timeline(mission, width=880):
    records = [r for r in (mission or {}).get("phases", []) if r.get("sim_start") is not None and r.get("sim_end") is not None]
    if not records:
        return '<p class="empty">The mission JSON has no phase times.</p>'
    t0, t1 = records[0]["sim_start"], max(r["sim_end"] for r in records)
    span = (t1 - t0) or 1.0
    left, w, row = 120, width - 136, 22
    out = [f'<svg class="chart timeline" viewBox="0 0 {width} {row * len(records) + 30}" role="img" aria-label="Phase timeline">']
    for i, r in enumerate(records):
        v = _verdict(r)
        x = left + (r["sim_start"] - t0) / span * w
        bw = max(2.0, (r["sim_end"] - r["sim_start"]) / span * w)
        y = 4 + i * row
        out.append(f'<text class="tick" x="{left - 8}" y="{y + 14}" text-anchor="end">{html.escape(r["phase"])}</text>'
                   f'<rect class="phase {VERDICT_TEXT.get(v, "fail").replace(" ", "-")}" x="{x:.1f}" y="{y}" width="{bw:.1f}" '
                   f'height="{row - 6}" rx="4"><title>{html.escape(r["phase"])}: {v}, {r["sim_end"] - r["sim_start"]:.1f} s</title></rect>'
                   + (f'<text class="tick" x="{x + bw + 6:.1f}" y="{y + 14}">{v}</text>' if x + bw + 60 < left + w else
                      f'<text class="tick" x="{x - 6:.1f}" y="{y + 14}" text-anchor="end">{v}</text>'))
    out.append(f'<text class="label" x="{left + w / 2:.1f}" y="{row * len(records) + 26}" text-anchor="middle">'
               f'Gazebo time, s ({t0:.1f} to {t1:.1f})</text></svg>')
    return "".join(out)


def phase_table(mission):
    rows = []
    for r in (mission or {}).get("phases", []):
        v = _verdict(r)
        duration = "-" if r.get("sim_start") is None or r.get("sim_end") is None else f'{r["sim_end"] - r["sim_start"]:.1f}'
        checks = [q for q in r.get("quantities", []) if q.get("ok") is not None]
        failed = "; ".join(f["text"] for f in r.get("failures", [])[:3])
        if r.get("exception"):
            failed = "; ".join(x for x in (failed, r["exception"].get("text", "")) if x)
        rows.append(f'<tr><td>{html.escape(r["phase"])}</td><td class="v {VERDICT_TEXT.get(v, "fail").replace(" ", "-")}">{v}</td>'
                    f'<td>{duration}</td><td>{sum(1 for q in checks if q["ok"])} of {len(checks)}</td>'
                    f'<td>{html.escape(failed or r.get("note", "") or "-")}</td></tr>')
    if not rows:
        return '<p class="empty">No phases in the mission JSON.</p>'
    return ('<table><thead><tr><th>Phase</th><th>Verdict</th><th>Gazebo s</th><th>Checks in range</th><th>Detail</th></tr>'
            '</thead><tbody>' + "".join(rows) + "</tbody></table>")


CSS = """
.viz-root{color-scheme:light;--surface-1:#fcfcfb;--surface-2:#f1f0ec;--text-primary:#0b0b0b;--text-secondary:#52514e;
--grid:#e2e1dc;--axis:#a3a29c;--good:#008300;--critical:#d32f2f;--warning:#b36b00;
""" + "".join(f"--series-{i + 1}:{c};" for i, c in enumerate(SERIES_LIGHT)) + """}
@media (prefers-color-scheme: dark){.viz-root{color-scheme:dark;--surface-1:#1a1a19;--surface-2:#242422;--text-primary:#f0efec;
--text-secondary:#c3c2b7;--grid:#34342f;--axis:#6b6a64;--good:#0ca30c;--critical:#e66767;--warning:#c98500;
""" + "".join(f"--series-{i + 1}:{c};" for i, c in enumerate(SERIES_DARK)) + """}}
body{margin:0;background:var(--surface-1);color:var(--text-primary);font:14px/1.45 system-ui,sans-serif}
main{max-width:920px;margin:0 auto;padding:24px 16px 48px}
h1{font-size:22px;margin:0 0 4px}h2{font-size:16px;margin:28px 0 6px}
.note,.empty,.meta{color:var(--text-secondary)}.meta{font-size:13px}
.chart{width:100%;height:auto;display:block}
.grid{stroke:var(--grid);stroke-width:1}.axis{stroke:var(--axis);stroke-width:1}
.tick{fill:var(--text-secondary);font-size:11px}.label{fill:var(--text-secondary);font-size:12px}
.line{fill:none;stroke-width:2;stroke-linejoin:round}
""" + "".join(f".s{i}{{stroke:var(--series-{i + 1});background:var(--series-{i + 1})}}" for i in range(8)) + """
.band0{fill:var(--surface-2)}.band1{fill:transparent}
.cross{stroke:var(--axis);stroke-dasharray:3 3}.hit{fill:transparent}
.plot{position:relative}.tip{position:absolute;top:8px;right:8px;background:var(--surface-2);border:1px solid var(--grid);
padding:6px 8px;border-radius:6px;font-size:12px;pointer-events:none;white-space:nowrap}
.legend{display:flex;flex-wrap:wrap;gap:4px 14px;font-size:12px;color:var(--text-secondary);margin:2px 0 4px}
.key{display:inline-block;width:14px;height:3px;border-radius:2px;margin-right:6px;vertical-align:middle}
.phase.pass{fill:var(--good)}.phase.fail{fill:var(--critical)}.phase.blocked,.phase.not-run{fill:var(--warning)}
table{border-collapse:collapse;width:100%;font-size:13px}th,td{text-align:left;padding:5px 8px;border-bottom:1px solid var(--grid);vertical-align:top}
td.v.pass{color:var(--good)}td.v.fail{color:var(--critical)}td.v.blocked,td.v.not-run{color:var(--warning)}
td.v::before{content:"\\25CF  "}
"""

SCRIPT = """
document.querySelectorAll('.plot').forEach(function(plot){
  var data=JSON.parse(plot.dataset.series),svg=plot.querySelector('svg'),hit=svg.querySelector('.hit'),
      cross=svg.querySelector('.cross'),tip=plot.querySelector('.tip');
  var x0=+hit.dataset.x0,x1=+hit.dataset.x1,left=+hit.dataset.left,w=+hit.dataset.w;
  hit.addEventListener('mousemove',function(e){
    var pt=svg.createSVGPoint();pt.x=e.clientX;pt.y=e.clientY;var p=pt.matrixTransform(svg.getScreenCTM().inverse());
    var x=x0+(p.x-left)/w*(x1-x0);cross.setAttribute('x1',p.x);cross.setAttribute('x2',p.x);cross.setAttribute('visibility','visible');
    var lines=[data.x+' '+x.toFixed(2)];
    data.series.forEach(function(s){var best=null;s.pts.forEach(function(q){if(best===null||Math.abs(q[0]-x)<Math.abs(best[0]-x))best=q;});
      if(best)lines.push(s.name+': '+best[1]);});
    tip.textContent='';lines.forEach(function(l,i){var d=document.createElement('div');d.textContent=l;tip.appendChild(d);});tip.hidden=false;});
  hit.addEventListener('mouseleave',function(){cross.setAttribute('visibility','hidden');tip.hidden=true;});
});
"""


def provenance_table(run_info):
    from openamrobot_control_room import state
    view = state.provenance(run_info, {}, [])
    rows = view.get("components")
    if not isinstance(rows, dict):
        return f'<p class="empty">{html.escape(str(rows))}</p>'
    body = "".join(f"<tr><td>{html.escape(k)}</td><td>{html.escape(str(v))}</td></tr>" for k, v in sorted(rows.items()))
    run = html.escape(json.dumps(view.get("run", {}), sort_keys=True))
    return (f"<table><thead><tr><th>Component</th><th>What it is in this run</th></tr></thead><tbody>{body}"
            f"<tr><td>command arbiter</td><td>{html.escape(view['command_arbiter'])}</td></tr>"
            f"<tr><td>hardware</td><td>{html.escape(view['hardware'])}</td></tr></tbody></table><p class=note>Run: {run}</p>")


def decoded_sections(decoded, section):
    if not decoded:
        section("Decoded CAN signals", '<p class="empty">No candump logs were given (--candump).</p>')
        return
    s = decoded["series"]
    wall = "host wall s since the first frame (candump time, not Gazebo time)"
    pick = lambda *names: [(n, s[n]) for n in names if n in s]  # noqa: E731
    section("Drive (vcan_drive, ZLAC8015D emulator): wheel speed", svg_chart(
        "can_drive", pick("drive left target (60FFh write)", "drive right target (60FFh write)",
                          "drive left actual (606Ch read)", "drive right actual (606Ch read)"), wall, "wheel rim m/s"),
        "SDO 60FFh writes (r/min) and 606Ch read replies (0.1 r/min) of the host stand-in, times 2 pi / 60 times "
        "the wheel radius 0.098 m of openamrobot_sil sil.yaml.")
    section("Lift (vcan_aux, CTL-017 emulator): position and velocity", svg_chart(
        "can_lift", pick("lift TPDO1 position", "lift TPDO2 velocity"), wall, "mm, mm/s"),
        "TPDO1 position (0.1 mm) and TPDO2 velocity (0.1 mm/s). Commands (RPDO1): " + (", ".join(
            f"t={c['t']} s seq {c['seq']} {c['op']} {c['stop'] or ''}" for c in decoded["commands"]) or "none"))
    arm = [n for n in sorted(s) if n.startswith("openarm_left_joint") and n.split()[0][-1] in "1247"]
    section("Left arm (vcan_arm_l, emulated Damiao motors): MIT command and feedback position", svg_chart(
        "can_arm", [(n, s[n]) for n in arm], wall, "rad"),
        "MIT command q of the unmodified openarm_ros2 driver and the motors' state feedback q, joints 1, 2, 4 and 7, "
        "every 25th frame of each ID.")
    section("BMS (vcan_bms, Daly emulator)", svg_chart("can_bms", pick("BMS SOC", "BMS pack voltage"), wall, "%, V"),
            "Data ID 90h replies. The Daly protocol is unconfirmed for the delivered unit.")
    rows = "".join(f"<tr><td>{html.escape(r['bus'])}</td><td>{html.escape(r['id'])}</td><td><code>{html.escape(r['data'])}"
                   f"</code></td><td>{html.escape(json.dumps(r['decoded'], sort_keys=True))}</td></tr>" for r in decoded["raw"])
    section("Selected raw frames: the first frame of each bus and ID", "<table><thead><tr><th>Bus</th><th>ID</th><th>Data"
            f"</th><th>Decoded</th></tr></thead><tbody>{rows}</tbody></table>")


def render(series, mission, can_rates=None, title="Baseline 1 mission report", source="", decoded=None):
    mission = mission or {}
    bands = phase_bands(mission)
    sim = "Gazebo time, s"
    sections = []

    def section(heading, body, note=""):
        sections.append(f"<section><h2>{html.escape(heading)}</h2>{f'<p class=note>{html.escape(note)}</p>' if note else ''}{body}</section>")
    section("Phase timeline", timeline(mission), "From the mission JSON (phases, sim_start, sim_end). The verdicts are the mission's.")
    section("Phase table", phase_table(mission))
    section("What ran: components of this run", provenance_table(series.get("run_info")),
            "From the run's /build1/run_info and the launch code (state.provenance). Emulators and stand-ins are not devices.")
    status_rows = "".join(f"<tr><td>{_fmt(t)}</td><td>{html.escape(topic)}</td><td>{html.escape(text)}</td></tr>"
                          for t, topic, text in series.get("mission_status", []))
    if status_rows:
        section("Mission status topic", "<table><thead><tr><th>Gazebo s</th><th>Topic</th><th>Message</th></tr></thead>"
                f"<tbody>{status_rows}</tbody></table>", "/baseline1/mission_status is Proposed (#82).")
    section("Station error", svg_chart("station", [("distance", series["station_error"]), ("along heading", series["station_along"]),
                                                   ("left of heading", series["station_left"])], sim, "m", bands),
            "Ground truth odometry against the work-shelf station pose. Shaded bands are the phases.")
    section("Localization error, position", svg_chart("loc", [("AMCL minus ground truth", series["loc_error"])], sim, "m", bands),
            "/amcl_pose against /build1/ground_truth/odom, both in the world frame (AMCL starts at the spawn pose).")
    section("Localization error, yaw", svg_chart("loc_yaw", [("AMCL minus ground truth", series["loc_yaw_error"])], sim, "rad", bands))
    for side in ("left", "right"):
        section(f"{side.capitalize()} arm joint states",
                svg_chart(f"joints_{side}", [(f"joint{i}", series["joints"][f"openarm_{side}_joint{i}"]) for i in range(1, 8)],
                          sim, "rad", bands), "/joint_states.")
    section("Lift position", svg_chart("lift", [("/joint_states", series["lift_js"]), ("Gazebo physics", series["lift_physics"])],
                                       sim, "lift_joint, m", bands), "/joint_states and /sil/gz/joint_states, lift_joint.")
    section("Item height", svg_chart("item", [("item z", series["item_z"])], sim, "m", bands), "/build1/ground_truth/item/pose.")
    section("Real-time factor", svg_chart("rtf", [("rtf_last", series["rtf"])], sim, "Gazebo s per wall s", bands),
            "/build1/rtf_stats (rtf_monitor.py, the last second).")
    rates = can_rates or {}
    section("CAN frame rate", svg_chart("can", [(bus, pts) for bus, pts in sorted(rates.items())],
                                        "wall s since the first frame", "frames per s"),
            "candump -L logs of the virtual buses (wall time, not Gazebo time)." if rates else
            "No candump logs were given (--candump).")
    decoded_sections(decoded, section)
    contacts = series["contacts"]
    top = sorted(contacts, key=lambda k: -max((f for _, f in contacts[k]), default=0.0))[:8]
    section("Contact force", svg_chart("contacts", [(k, contacts[k]) for k in top], sim, "N", bands),
            "Largest total force per contact sensor topic. Contact sensors exist only with BUILD1_CONTACT_SENSORS=1.")
    topics = "".join(f"<tr><td>{html.escape(t)}</td><td>{n}</td></tr>" for t, n in sorted(series["topics"].items()))
    section("Recorded topics", f"<table><thead><tr><th>Topic</th><th>Messages</th></tr></thead><tbody>{topics}</tbody></table>"
            if topics else '<p class="empty">The recording has no messages.</p>')
    span = series.get("sim_span")
    meta = [f"Mission result: {html.escape(str(mission.get('result')))}" + (f" ({html.escape(str(mission.get('reason')))})" if mission.get("reason") else ""),
            f"Settings: {html.escape(json.dumps(mission.get('baseline1', {}), sort_keys=True))}",
            f"Gazebo time span of the bag: {_fmt(span[0])} to {_fmt(span[1])} s" if span else "No /clock in the bag",
            f"Source: {html.escape(source)}" if source else ""]
    return ("<!DOCTYPE html><html lang=en><head><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
            f"<title>{html.escape(title)}</title><style>{CSS}</style></head><body class=viz-root><main>"
            f"<h1>{html.escape(title)}</h1><p class=meta>{'<br>'.join(m for m in meta if m)}</p>"
            "<p class=note>Simulation record only. Not hardware, physical acceptance or safety evidence. "
            "The verdicts come from the mission JSON; this page computes none.</p>"
            + "".join(sections) + f"</main><script>{SCRIPT}</script></body></html>")
