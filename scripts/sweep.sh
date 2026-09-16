#!/usr/bin/env bash
# Baseline sweep on the stock 5G-LENA highway scenario: node density x sensing x seed.
# Run inside the dev container:  docker exec slv2x bash /sidewalk/scripts/sweep.sh
# One SQLite KPI db per run lands in /sidewalk/results/<tag>/.
set -euo pipefail
cd /opt/ns-3-dev
TAG=${TAG:-baseline}
OUT=/sidewalk/results/$TAG; mkdir -p "$OUT"
LANES=${LANES:-3}
PER_LANE=${PER_LANE:-"5 10 15 20"}       # x3 lanes -> 15..60 nodes
RUNS=${RUNS:-"1 2 3 4 5"}
SIM_TIME=${SIM_TIME:-5}
JOBS=${JOBS:-4}

run_one() {  # per_lane sensing run
  local n=$1 s=$2 r=$3
  local tag="n${n}_s${s}_r${r}"
  [ -s "$OUT/${tag}-nr-v2x-west-to-east-highway.db" ] && return 0
  ./build/contrib/nr/examples/ns3.42-nr-v2x-west-to-east-highway-optimized \
      --numVehiclesPerLane=$n --numLanes=$LANES --enableOneTxPerLane=0 \
      --enableSensing=$s --simTime=$SIM_TIME --RngRun=$r \
      --simTag=$tag --outputDir="$OUT/" > "$OUT/${tag}.log" 2>&1 \
    && echo "done $tag" || echo "FAILED $tag (see $OUT/$tag.log)"
}
export -f run_one; export OUT LANES SIM_TIME
for n in $PER_LANE; do for s in 0 1; do for r in $RUNS; do echo "$n $s $r"; done; done; done \
  | xargs -P "$JOBS" -n 3 bash -c 'run_one "$@"' _
