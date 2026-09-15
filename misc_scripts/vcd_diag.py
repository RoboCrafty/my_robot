import re, statistics as st

VCD = '/home/parol-rpi/my_robot/wokwi.vcd'

# Map every $var to its symbol + human name
sym2name = {}
with open(VCD) as f:
    for line in f:
        line = line.strip()
        if line.startswith('$var'):
            p = line.split()
            sym2name[p[3]] = p[4]           # symbol -> name (e.g. D4)
        elif line.startswith('$enddefinitions'):
            break

# Collect rising-edge timestamps per symbol
edges = {s: [] for s in sym2name}
t = 0
with open(VCD) as f:
    for line in f:
        line = line.strip()
        if not line:
            continue
        if line[0] == '#':
            t = int(line[1:])
        elif len(line) == 2 and line[0] in '01':
            val, sym = line[0], line[1]
            if sym in edges and val == '1':
                edges[sym].append(t)

# Rank channels by rising-edge count
ranked = sorted(edges.items(), key=lambda kv: len(kv[1]), reverse=True)
print("channel activity (rising edges):")
for s, ts in ranked[:10]:
    print(f"  {sym2name.get(s,'?'):>5} ({s}) : {len(ts)}")

# Analyse the busiest channel
sym, ts = ranked[0]
print(f"\n--- analysing {sym2name.get(sym)} ({sym}), {len(ts)} pulses ---")
if len(ts) < 3:
    raise SystemExit("not enough pulses")

# VCD timescale? assume 1us (Wokwi default per the analyser)
dts = [ts[i]-ts[i-1] for i in range(1, len(ts))]
dts_us = [d for d in dts if d > 0]
print(f"total span: {(ts[-1]-ts[0])/1e6:.3f} s")
print(f"dt (us): min={min(dts_us)} median={st.median(dts_us):.1f} max={max(dts_us)} mean={st.mean(dts_us):.1f}")

# Burstiness: fraction of intervals that are near-zero (batched) vs the median
med = st.median(dts_us)
near_zero = sum(1 for d in dts if d <= max(1, med*0.05))
huge      = sum(1 for d in dts if d >= med*5)
print(f"intervals <=5% of median: {near_zero} ({100*near_zero/len(dts):.1f}%)   >=5x median: {huge} ({100*huge/len(dts):.1f}%)")

# How many distinct dt values (quantization)?
from collections import Counter
c = Counter(dts)
print(f"distinct dt values: {len(c)}; top 8: {c.most_common(8)}")

# Sample the velocity trace across the move (every ~5% of pulses)
print("\ntime(s)   dt(us)   inst_vel(steps/s)")
step = max(1, len(ts)//25)
for i in range(1, len(ts), step):
    d = ts[i]-ts[i-1]
    v = 1e6/d if d>0 else 0
    print(f"  {ts[i]/1e6:7.3f}  {d:7d}   {v:8.1f}")
