#!/usr/bin/env python3
"""Aggregate v2x-kpi SQLite outputs from scripts/sweep.sh into one CSV + figure.

Usage: analysis/kpi.py results/<tag> [--pdb-ms 20] [--lanes 3]
Reads every  n{N}_s{S}_r{R}-*.db  in the directory (N vehicles/lane, S sensing, R RngRun).
"""
import argparse, glob, os, re, sqlite3
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

def one_run(db, pdb_s):
    c = sqlite3.connect(db)
    q = lambda s: pd.read_sql_query(s, c)
    prr = q("select avrgPrr from avrgPrr")["avrgPrr"].mean()
    pir = q("select avrgPirSec from avrgPir")["avrgPirSec"].mean()
    st = q("select * from simulPsschTx").iloc[0]
    tb = q("select * from PsschTbRx").iloc[0]
    # per-packet latency: match rx rows to the tx row with same (srcIp, pktSeqNum)
    p = q("select timeSec, txRx, srcIp, pktSeqNum from pktTxRx")
    tx = p[p.txRx == "tx"].set_index(["srcIp", "pktSeqNum"])["timeSec"]
    rx = p[p.txRx == "rx"]
    lat = (rx.timeSec.values - tx.reindex(list(zip(rx.srcIp, rx.pktSeqNum))).values)
    lat = lat[~np.isnan(lat)]
    c.close()
    return dict(prr=prr, pir_ms=1e3 * pir,
                collision_frac=st.numOverlapping / st.totalTx,
                pssch_fail_frac=tb.psschFailCount / tb.totalRx,
                lat_p50_ms=1e3 * np.percentile(lat, 50), lat_p95_ms=1e3 * np.percentile(lat, 95),
                within_pdb=(lat <= pdb_s).mean(), n_rx=len(lat))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir"); ap.add_argument("--pdb-ms", type=float, default=20)
    ap.add_argument("--lanes", type=int, default=3)
    a = ap.parse_args()
    rows = []
    for db in sorted(glob.glob(os.path.join(a.dir, "n*_s*_r*-*.db"))):
        m = re.match(r"n(\d+)_s(\d)_r(\d+)-", os.path.basename(db))
        n, s, r = map(int, m.groups())
        rows.append(dict(nodes=n * a.lanes, scheme="sensing" if s else "random", run=r,
                         **one_run(db, a.pdb_ms / 1e3)))
    df = pd.DataFrame(rows)
    df.to_csv(os.path.join(a.dir, "runs.csv"), index=False)
    g = df.groupby(["scheme", "nodes"]).agg(["mean", "std"]).drop(columns="run")
    g.to_csv(os.path.join(a.dir, "summary.csv"))
    print(g[[("prr", "mean"), ("pir_ms", "mean"), ("collision_frac", "mean"),
             ("lat_p95_ms", "mean"), ("within_pdb", "mean")]].round(3).to_string())

    panels = [("prr", "PRR"), ("pir_ms", "mean PIR [ms]"),
              ("collision_frac", "fraction of PSSCH tx overlapping"),
              ("lat_p95_ms", "p95 latency [ms]")]
    fig, axs = plt.subplots(2, 2, figsize=(9, 6.5), constrained_layout=True)
    for ax, (k, label) in zip(axs.flat, panels):
        for scheme, mk in (("random", "o--"), ("sensing", "s-")):
            d = g.loc[scheme]
            ax.errorbar(d.index, d[(k, "mean")], yerr=d[(k, "std")], fmt=mk, capsize=3, label=scheme)
        ax.set_xlabel("nodes"); ax.set_ylabel(label); ax.grid(alpha=.3)
    axs.flat[0].legend(title="Mode 2 selection")
    fig.suptitle(f"5G-LENA NR sidelink Mode 2, stock highway scenario ({df.run.nunique()} seeds, mean ± std)")
    out = os.path.join(a.dir, "kpi.png"); fig.savefig(out, dpi=150); print("wrote", out)

if __name__ == "__main__":
    main()
