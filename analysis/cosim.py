#!/usr/bin/env python3
"""Validate the ROS 2 co-simulation against a standalone run of the same configuration.
"""
import sqlite3, sys, os
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

csv, db = sys.argv[1], sys.argv[2]
co = pd.read_csv(csv)
co_lat = co.latency_us.values / 1e3
n = int(max(co.src.max(), co.dst.max())) + 1

c = sqlite3.connect(db)
p = pd.read_sql_query("select timeSec, txRx, srcIp, pktSeqNum from pktTxRx", c)
tx, rx = p[p.txRx == "tx"], p[p.txRx == "rx"]
sa_lat = (rx.timeSec.values - tx.set_index(["srcIp", "pktSeqNum"])["timeSec"]
          .reindex(list(zip(rx.srcIp, rx.pktSeqNum))).values) * 1e3
sa_lat = sa_lat[~np.isnan(sa_lat)]
sa_prr = len(rx) / (len(tx) * (n - 1))

def pct(a, q): return np.percentile(a, q)
print(f"{'':12s} {'n':>7s} {'p50':>6s} {'p95':>6s} {'p99':>6s}")
print(f"{'co-sim':12s} {len(co_lat):7d} {pct(co_lat,50):6.1f} {pct(co_lat,95):6.1f} {pct(co_lat,99):6.1f} ms")
print(f"{'standalone':12s} {len(sa_lat):7d} {pct(sa_lat,50):6.1f} {pct(sa_lat,95):6.1f} {pct(sa_lat,99):6.1f} ms   PRR={sa_prr:.3f}")

fig, ax = plt.subplots(figsize=(6, 3.8), constrained_layout=True)
for lat, label, ls in ((sa_lat, "standalone ns-3 (OnOff traffic, random waypoint)", "-"),
                       (co_lat, "co-simulation (ROS 2 robots drive mobility + emission)", "--")):
    x = np.sort(lat); ax.plot(x, np.arange(1, len(x) + 1) / len(x), ls, label=f"{label}, n={len(x)}")
ax.set_xlabel("state-message latency [ms]"); ax.set_ylabel("CDF"); ax.grid(alpha=.3)
ax.set_xlim(0, max(pct(co_lat, 99.5), pct(sa_lat, 99.5))); ax.legend(fontsize=8, loc="lower right")
ax.set_title(f"{n} robots, 100 ms messages, sensing on, 1 retx / 10-RB subch / μ=1", fontsize=10)
out = os.path.join(os.path.dirname(csv), "latency_cdf.png"); fig.savefig(out, dpi=150); print("wrote", out)
