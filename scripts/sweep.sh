#!/usr/bin/env bash
# Generic sweep driver that runs inside the dev container:
#   docker exec slv2x bash /sidewalk/scripts/sweep.sh <preset>
set -euo pipefail
cd /opt/ns-3-dev
PRESET=${1:?usage: sweep.sh <robots-density|robots-period>}
OUT=/sidewalk/results/$PRESET; mkdir -p "$OUT"
RUNS=${RUNS:-"1 2 3 4 5"}; JOBS=${JOBS:-4}; SIM_TIME=${SIM_TIME:-5}
ROBOTS=./build/contrib/sidewalk/examples/ns3.42-sidewalk-robots-optimized

jobs() {
  case $PRESET in
    robots-density)  # robots in 50 m, 100 ms state messages
      for n in 5 10 20 40; do for s in 0 1; do for r in $RUNS; do
        echo "nodes=${n}_scheme=${s}_run=$r|$ROBOTS --numRobots=$n --enableSensing=$s"
      done; done; done ;;
    robots-period)   # 20 robots, message period sweep (SPS period follows; must be a multiple of the 20-slot pool)
      for p in 20 40 100; do for s in 0 1; do for r in $RUNS; do
        echo "period=${p}_scheme=${s}_run=$r|$ROBOTS --numRobots=20 --msgPeriod=$p --enableSensing=$s"
      done; done; done ;;
    *) echo "unknown preset $PRESET"; exit 1 ;;
  esac
}

run_one() {
  local tag=${1%%|*} cmd=${1#*|}
  compgen -G "$OUT/${tag}-*.db" > /dev/null && [ -s "$(compgen -G "$OUT/${tag}-*.db" | head -1)" ] && return 0
  $cmd --simTime=$SIM_TIME --RngRun=${tag##*run=} --simTag="$tag" --outputDir="$OUT/" > "$OUT/$tag.log" 2>&1 \
    && echo "done $tag" || echo "FAILED $tag (see $OUT/$tag.log)"
}
export -f run_one; export OUT SIM_TIME
jobs | xargs -P "$JOBS" -d '\n' -n 1 bash -c 'run_one "$@"' _
