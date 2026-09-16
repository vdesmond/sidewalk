#!/usr/bin/env python3
"""Aggregate one sweep directory (from scripts/sweep.sh) into runs.csv, summary.csv and figures.

Usage: analysis/kpi.py results/<preset> [--pdb-ms 20] [--title "..."]
"""
import argparse, glob, os, re, sqlite3
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.style.use(os.path.join(os.path.dirname(os.path.abspath(__file__)), "style.mplstyle"))

SCHEMES = {0: ("random", "o--"), 1: ("sensing", "s-")}

def kpis(db, pdb_s):
    c = sqlite3.connect(db)
    q = lambda s: pd.read_sql_query(s, c)
    p = q("select timeSec, txRx, srcIp, pktSeqNum from pktTxRx")
    tx, rx = p[p.txRx == "tx"], p[p.txRx == "rx"]
    n_nodes = p.srcIp.nunique()
    # upstream avrgPrr counts the transmitter as its own (never receiving) neighbour, so
    # compute PRR directly: every packet has n_nodes-1 potential receivers.
    prr = len(rx) / (len(tx) * (n_nodes - 1))
    pir = q("select avrgPirSec from avrgPir")["avrgPirSec"].mean()
    st = q("select * from simulPsschTx").iloc[0]
    tb = q("select * from PsschTbRx").iloc[0]
    lat = rx.timeSec.values - tx.set_index(["srcIp", "pktSeqNum"])["timeSec"] \
        .reindex(list(zip(rx.srcIp, rx.pktSeqNum))).values
    lat = lat[~np.isnan(lat)]
    c.close()
    return dict(prr=prr, pir_ms=1e3 * pir,
                collision_frac=st.numOverlapping / st.totalTx,
                pssch_fail_frac=tb.psschFailCount / max(tb.totalRx, 1),
                lat_p50_ms=1e3 * np.percentile(lat, 50), lat_p95_ms=1e3 * np.percentile(lat, 95),
                within_pdb=(lat <= pdb_s).mean())

def sensing(csv):
    if not os.path.exists(csv):
        return {}
    d = pd.read_csv(csv).dropna(subset=["rsrp_thr_init_dbm"])
    if d.empty:
        return {}
    # step 4 -> 5: candidates dropped because the UE itself transmits there (half-duplex);
    # step 5 -> output: candidates dropped by decoded-SCI + RSRP-threshold exclusion (step 6).
    return dict(sel_count=len(d), backoff_frac=(d.backoff_steps > 0).mean(),
                backoff_steps_mean=d.backoff_steps.mean(),
                hd_excl_frac=1 - (d.s_a_step5 / d.s_a_step4).mean(),
                rsrp_excl_frac=1 - (d.cand_out / d.s_a_step5).mean())

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir"); ap.add_argument("--pdb-ms", type=float, default=20)
    ap.add_argument("--title", default=None)
    ap.add_argument("--order", default=None, help="comma-separated order of categorical x values")
    a = ap.parse_args()
    rows = []
    for db in sorted(glob.glob(os.path.join(a.dir, "*.db"))):
        base = os.path.basename(db)
        # "<tags>-<example>.db": tags end at the first "-" that follows a "_run=<n>" group
        tagstr = re.match(r"(.*?_run=\d+)-", base).group(1)
        tags = dict(kv.split("=") for kv in tagstr.split("_"))
        tags = {k: (float(v) if re.fullmatch(r"-?\d+(\.\d+)?", v) else v) for k, v in tags.items()}
        rows.append({**tags, **kpis(db, a.pdb_ms / 1e3), **sensing(db[:-3] + "-sensing.csv")})
    df = pd.DataFrame(rows)
    x = next(k for k in rows[0] if k not in ("scheme", "run"))
    df["scheme"] = df.scheme.astype(int).map(lambda s: SCHEMES[s][0])
    df.to_csv(os.path.join(a.dir, "runs.csv"), index=False)
    g = df.drop(columns="run").groupby(["scheme", x]).agg(["mean", "std"])
    g.to_csv(os.path.join(a.dir, "summary.csv"))
    show = ["prr", "collision_frac", "lat_p50_ms", "lat_p95_ms", "within_pdb"] + \
           (["hd_excl_frac", "rsrp_excl_frac", "backoff_frac"] if "backoff_frac" in df else [])
    print(g[[(k, "mean") for k in show]].round(3).to_string())

    panels = [("prr", "PRR"), ("collision_frac", "fraction of PSSCH tx overlapping"),
              ("lat_p95_ms", "p95 latency [ms]" if a.pdb_ms >= 20 else "p50 latency [ms]"),
              ("within_pdb", f"fraction within PDB ({a.pdb_ms:g} ms)")]
    if a.pdb_ms < 20:
        panels[2] = ("lat_p50_ms", "median latency [ms]")
    cats = sorted(df[x].unique(), key=lambda v: (isinstance(v, str), v))
    if a.order:
        cats = [c for c in a.order.split(",") if c in cats] + [c for c in cats if c not in a.order.split(",")]
    categorical = any(isinstance(v, str) for v in cats)
    xpos = lambda idx: [cats.index(v) for v in idx] if categorical else idx
    # no connecting lines between unordered categories
    fmt = lambda mk: mk.rstrip("-:.") if categorical else mk

    def style(ax):
        ax.set_xlabel(x); ax.grid(alpha=.3)
        if categorical:
            ax.set_xticks(range(len(cats))); ax.set_xticklabels(cats, rotation=20, ha="right", fontsize=8)

    fig, axs = plt.subplots(2, 2, figsize=(9, 6.5), constrained_layout=True)
    for ax, (k, label) in zip(axs.flat, panels):
        for s, (name, mk) in SCHEMES.items():
            if name not in g.index.get_level_values(0):
                continue
            d = g.loc[name]
            ax.errorbar(xpos(d.index), d[(k, "mean")], yerr=d[(k, "std")], fmt=fmt(mk), capsize=3, label=name)
        ax.set_ylabel(label); style(ax)
    axs.flat[0].legend(title="Mode 2 selection")
    fig.suptitle(a.title or f"{os.path.basename(a.dir.rstrip('/'))} ({df.run.nunique()} seeds, mean ± std)")
    out = os.path.join(a.dir, "kpi.png"); fig.savefig(out, dpi=150); print("wrote", out)

    if "backoff_frac" in df:
        d = g.loc["sensing"]
        fig, axs = plt.subplots(1, 2, figsize=(9, 3.4), constrained_layout=True)
        axs[0].errorbar(xpos(d.index), d[("rsrp_excl_frac", "mean")], yerr=d[("rsrp_excl_frac", "std")], fmt=fmt("s-"), capsize=3, label="others' reservations (SCI + RSRP, step 6)")
        axs[0].errorbar(xpos(d.index), d[("hd_excl_frac", "mean")], yerr=d[("hd_excl_frac", "std")], fmt=fmt("o:"), capsize=3, label="own transmissions (half-duplex, step 5)")
        axs[0].set_ylabel("fraction of candidate resources excluded"); axs[0].legend(fontsize=8)
        axs[1].errorbar(xpos(d.index), d[("backoff_frac", "mean")], yerr=d[("backoff_frac", "std")], fmt=fmt("s-"), capsize=3, label="fraction of selections")
        axs[1].errorbar(xpos(d.index), d[("backoff_steps_mean", "mean")], yerr=d[("backoff_steps_mean", "std")], fmt=fmt("^:"), capsize=3, label="mean 3 dB steps")
        axs[1].set_ylabel("RSRP-threshold back-off (step 6)"); axs[1].legend()
        for ax in axs: style(ax)
        fig.suptitle("Mode 2 sensing algorithm (TS 38.214 §8.1.4) internals, sensing enabled")
        out = os.path.join(a.dir, "sensing.png"); fig.savefig(out, dpi=150); print("wrote", out)

if __name__ == "__main__":
    main()
