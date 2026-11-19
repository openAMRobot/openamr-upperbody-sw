# SPDX-License-Identifier: MIT
"""Baseline 1 bag, extra series for Video C: arm controller tracking error, lift emulator clock counters."""
import sys, json
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader
E, K = [], []
with AnyReader([Path(sys.argv[1])]) as r:
    conns = [c for c in r.connections if c.topic in ("/left_arm_controller/controller_state", "/sil/lift/emulator_state")]
    last = -1
    for c, t, raw in r.messages(connections=conns):
        t /= 1e9
        if c.topic.startswith("/left"):
            if int(t * 25) == last: continue
            last = int(t * 25); m = r.deserialize(raw, c.msgtype)
            E.append((t, *list(m.error.positions)[:7], *list(m.reference.positions)[:7], *list(m.feedback.positions)[:7]))
        else:
            d = json.loads(r.deserialize(raw, c.msgtype).data); sc = d["sim_clock"]
            K.append((t, sc["now"], sc["stalls"], sc["jumps"], sc["resets"], d["clock_stalls"], d["permit_rx_count"], d["true_mm"], d["position_mm"]))
np.savez_compressed(sys.argv[2], err=np.array(E), clk=np.array(K)); print(len(E), len(K), E[0][:3], K[0])
