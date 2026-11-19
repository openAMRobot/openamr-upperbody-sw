"""Scratch analysis of recorded Baseline 1 artifacts (not for merge). Prints evidence lines only."""
import glob, json, os, re, struct, sys
LINE = re.compile(r"\((\d+\.\d+)\)\s+(\S+)\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)")

def frames(path):
    with open(path, errors="replace") as f:
        for line in f:
            m = LINE.match(line.strip())
            if m:
                yield float(m.group(1)), int(m.group(3), 16), bytes.fromhex(m.group(4))

def drive(path, t_death):
    print("== drive", path)
    req = None; n = 0; mism = []; to = 0; last = []
    pend = []
    for t, cid, d in frames(path):
        n += 1
        if cid == 0x601:
            pend.append((t, d))
        elif cid == 0x581 and d:
            idx = struct.unpack("<HB", d[1:4]) if len(d) >= 4 else None
            if pend:
                tq, q = pend.pop(0)
                qidx = struct.unpack("<HB", q[1:4])
                if qidx != idx or (q[0] == 0x40) != (d[0] in (0x4F, 0x4B, 0x43, 0x80)):
                    mism.append((t, tq, q.hex(), d.hex()))
        if t_death - 3 <= t <= t_death + 0.5:
            last.append("%.6f %03X %s" % (t, cid, d.hex()))
    print("frames", n, "unanswered requests at end", len(pend))
    print("first request/response mismatches (pairing in order):")
    for m in mism[:12]:
        print("  resp t=%.6f req t=%.6f req=%s resp=%s" % m)
    print("mismatch count", len(mism))
    print("trace around", t_death, ":")
    for l in last[-80:]:
        print("  ", l)
    # latency of SDO responses
    pend = []; lat = []
    for t, cid, d in frames(path):
        if cid == 0x601: pend.append(t)
        elif cid == 0x581 and pend: lat.append((t - pend.pop(0), t))
    lat.sort(reverse=True)
    print("largest request-to-response gaps (s, at):", [(round(a, 4), round(b, 3)) for a, b in lat[:15]])

def aux(path, t0, t1):
    print("== aux", path, t0, t1)
    for t, cid, d in frames(path):
        if t0 <= t <= t1 and cid in (0x210, 0x410, 0x190, 0x290, 0x090, 0x710, 0x000):
            extra = ""
            if cid == 0x190 and len(d) == 8:
                s = struct.unpack("<BBHBBBB", d); extra = "state %d flags %02X pos %d stop %d last_seq %d res %02X cnt %d" % s
            if cid == 0x210:
                extra = "cmd seq %d op %d stop %d" % (d[0], d[1], d[2]) if len(d) >= 3 else ""
            if cid == 0x410 and len(d) >= 5:
                extra = "permit flag %d boot %04X seq %d" % (d[0], struct.unpack('<H', d[1:3])[0], struct.unpack('<H', d[3:5])[0])
            if cid == 0x290 and len(d) == 8:
                s = struct.unpack("<BBHhBB", d); extra = "in %02X target %d cur %d vel %d faults %02X aux %02X" % s
            print("  %.6f %03X %-18s %s" % (t, cid, d.hex(), extra))

root = sys.argv[1]
for p in sorted(glob.glob(root + "/**/*", recursive=True))[:400]:
    if os.path.isfile(p): print("file", os.path.relpath(p, root), os.path.getsize(p))
mode = sys.argv[2]
if mode == "drive":
    for p in glob.glob(root + "/**/*vcan_drive*", recursive=True): drive(p, float(sys.argv[3]))
elif mode == "aux":
    for p in glob.glob(root + "/**/*vcan_aux*", recursive=True):
        aux(p, float(sys.argv[3]), float(sys.argv[4]))
for p in glob.glob(root + "/**/baseline1_mission.json", recursive=True):
    data = json.load(open(p))
    text = json.dumps(data)
    print("mission json keys", list(data)[:30] if isinstance(data, dict) else type(data))
    for key in ("permit", "refus", "cmd_result", "sdo_errors", "emulator_state", "host_client"):
        for m in re.finditer(r'.{0,160}%s.{0,200}' % key, text):
            print("json", key, ":", m.group(0)); break
