# sidewalk

5G-NR sidelink Mode 2 resource allocation for cooperative robots — built on ns-3 / 5G-LENA,
with ROS 2 co-simulation via ns3-cosim.

Robots that coordinate over PC5 without a base station have to pick their own radio resources.
Mode 2 does this with **sensing-based semi-persistent scheduling** (TS 38.214 §8.1.4, TS 38.321 §5.22):
listen for other UEs' reservations, exclude what's taken, pick from what's left, keep it for a while.
This repo measures how well that works under robot-like traffic and tries to do better.

## Status

- [x] Pinned, reproducible stack (arm64 + x86): CTTC ns-3 fork `ns-3-dev-v2x-v1.1` · 5G-LENA `nr` `v2x-1.1` · NIST `ns3-cosim`
- [x] Baseline sweep on the stock highway scenario: random vs sensing-based selection, node density × 5 seeds
- [ ] Indoor cooperative-robotics scenario (small area, periodic short state messages, tight PDB)
- [ ] Sensing-algorithm instrumentation (RSRP threshold back-off steps vs density) via the `SensingAlgorithm` trace
- [ ] Own Mode 2 scheduler variant (`NrSlUeMacSchedulerFixedMcs::DoNrSlAllocation` override)
- [ ] ROS 2 co-simulation: `rclcpp` robots ↔ ns3-cosim gateway ↔ 5G-LENA

## First result

Stock `nr-v2x-west-to-east-highway`, 3 lanes, 200 B every 100 ms per vehicle, 5 s, 5 seeds:

![baseline](results/baseline/kpi.png)

Sensing buys 8–17 pp of PRR and roughly halves overlapping transmissions at low density; the
gain collapses above ~45 nodes when the resource pool is simply full. p95 latency sits near the
100 ms reservation period for both schemes — packets wait for their next SPS grant — which is the
axis to attack for ultra-low latency, not the air interface itself.

## Layout

```
docker/     setup.sh (verified build steps), Dockerfile, dev.sh (persistent dev container)
scripts/    sweep.sh — density × sensing × seed sweep, one KPI SQLite db per run
analysis/   kpi.py  — aggregates the dbs → runs.csv, summary.csv, kpi.png
results/    figures + CSVs are committed, .db/.log are not
```

## Reproduce

```bash
bash docker/dev.sh                                   # container `slv2x`, build on volume `slv2x-opt` (~40 min, -j4, 9 GB)
docker exec slv2x bash /sidewalk/scripts/sweep.sh    # ~10 min on 4 cores
python3 -m venv .venv && .venv/bin/pip install matplotlib pandas
.venv/bin/python analysis/kpi.py results/baseline
```

Gotchas found on the way: the `nr` tag is `v2x-1.1` (the README says `v2x-v1.1`); the highway example
aborts on an even `numVehiclesPerLane` unless `--enableOneTxPerLane=0`; the optimized build profile has
`NS_LOG` compiled out, so the ns3-cosim examples run silently; a `-j10` build OOM-killed a 12 GB VM.

## Metrics (from `v2x-kpi`)

`avrgPrr` packet reception ratio · `avrgPir` packet inter-reception time · `simulPsschTx` overlapping
transmissions (collisions) · `PsschTbRx` transport-block success/fail · per-packet latency and
PDB compliance from `pktTxRx` (tx/rx rows matched on source IP + sequence number).
