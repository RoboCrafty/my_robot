import re, statistics as st
from collections import Counter

VCD = '/home/parol-rpi/my_robot/wokwi.vcd'

# --- parse timescale + var map ---
ts_ns = 1.0
sym2name = {}
with open(VCD) as f:
    for line in f:
        s = line.strip()
        if s.startswith('$timescale'):
            m = re.search(r'(\d+)\s*(ns|us|ms|ps|s)', s)
            if m:
                unit = {'ps':1e-3,'ns':1,'us':1e3,'ms':1e6,'s':1e9}[m.group(2)]
                ts_ns = int(m.group(1))*unit
        elif s.startswith('$var'):
            p = s.split(); sym2name[p[3]] = p[4]
        elif s.startswith('$enddefinitions'):
            break
print(f"timescale: {ts_ns:.0f} ns/tick")

# --- collect rising edges per symbol (in ns) ---
edges = {s: [] for s in sym2name}
t = 0
with open(VCD) as f:
    for line in f:
        if not line: continue
        c = line[0]
        if c == '#':
            t = int(line[1:]) * ts_ns
        elif line[0] in '01' and len(line.strip()) == 2:
            val, sym = line[0], line[1]
            if val == '1' and sym in edges:
                edges[sym].append(t)

ranked = sorted(edges.items(), key=lambda kv: len(kv[1]), reverse=True)
print("\nrising edges per channel:")
for s, e in ranked:
    print(f"  {sym2name.get(s,'?'):>4} : {len(e)}")

sym, ns = ranked[0]
print(f"\n--- {sym2name.get(sym)} : {len(ns)} pulses ---")
if len(ns) < 5: raise SystemExit

dt = [ns[i]-ns[i-1] for i in range(1, len(ns))]        # ns between steps
dtp = [d for d in dt if d > 0]
print(f"span: {(ns[-1]-ns[0])/1e9:.3f} s")
print(f"dt(us): min={min(dtp)/1e3:.1f} med={st.median(dtp)/1e3:.1f} max={max(dtp)/1e3:.1f}")
peak_v = 1e9/min(dtp)
print(f"peak inst velocity: {peak_v:.0f} steps/s")

med = st.median(dtp)
near0 = sum(1 for d in dt if 0 < d <= med*0.1)
huge  = sum(1 for d in dt if d >= med*5)
zero  = sum(1 for d in dt if d == 0)
print(f"dt==0 (simultaneous): {zero}   <=10%*med: {near0} ({100*near0/len(dt):.1f}%)   >=5x med: {huge} ({100*huge/len(dt):.1f}%)")

# distinct dt (quantization signature)
c = Counter(round(d/1e3, 1) for d in dtp)               # us, 0.1us bins
print(f"distinct dt (0.1us bins): {len(c)}; top: {c.most_common(6)}")

# velocity profile across the whole move
print("\n t(s)     dt(us)   v(steps/s)")
stepn = max(1, len(ns)//40)
for i in range(1, len(ns), stepn):
    d = ns[i]-ns[i-1]
    v = 1e9/d if d > 0 else 0
    print(f" {ns[i]/1e9:6.3f}  {d/1e3:8.1f}  {v:9.0f}")

# --- fine structure: find the first sustained acceleration ramp and dump it ---
# locate first index where a burst of small-ish dt begins (a move start)
start = None
for i in range(1, len(ns)):
    if dt[i-1] < med*20:            # entered a move
        start = i; break
print(f"\n--- consecutive steps from first move (idx {start}) ---")
print(" idx   t(ms)    dt(us)   v(steps/s)   d(v) ")
prev_v = None
for i in range(start, min(start+80, len(ns))):
    d = ns[i]-ns[i-1]
    v = 1e9/d if d > 0 else 0
    dv = "" if prev_v is None else f"{v-prev_v:+8.0f}"
    print(f" {i:4d}  {ns[i]/1e6:8.2f}  {d/1e3:8.1f}  {v:9.0f}   {dv}")
    prev_v = v

# --- effective setpoint rate: count constant-rate segments during active motion ---
# A "segment" = a run of consecutive intervals with ~equal dt (one moveTimed cmd).
seg = 0
active_time_ns = 0
i = 1
while i < len(dt):
    d0 = dt[i-1]
    if d0 > med*20:      # idle gap between moves -> skip
        i += 1; continue
    j = i
    while j < len(dt) and abs(dt[j-1]-d0) <= max(200.0, d0*0.02):
        active_time_ns += dt[j-1]; j += 1
    seg += 1; i = j
if active_time_ns > 0:
    print(f"\nconstant-velocity segments: {seg} over {active_time_ns/1e9:.3f}s of motion")
    print(f"==> effective setpoint rate ~= {seg/(active_time_ns/1e9):.1f} Hz "
          f"(expected ~500 Hz)  | mean tread = {active_time_ns/1e9/seg*1000:.1f} ms")
