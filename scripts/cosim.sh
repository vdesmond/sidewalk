#!/usr/bin/env bash
# End-to-end co-simulation inside the dev container:
#   docker exec slv2x bash /sidewalk/scripts/cosim.sh [num_robots] [period_ms] [duration_s] [extra sidewalk-robots args...]
# ROS 2 (robots + bridge) is launched first, then ns-3 connects to the bridge.
set -eo pipefail   # no -u: the ROS setup scripts reference unset variables
N=${1:-10}; PERIOD=${2:-100}; DUR=${3:-20}; shift $(( $# < 3 ? $# : 3 )) || true
PORT=8000
CSV=/sidewalk/results/cosim/n${N}_p${PERIOD}.csv
mkdir -p "$(dirname "$CSV")"

source /opt/ros/humble/setup.bash
source /opt/ros2_ws/install/setup.bash
ros2 launch sidewalk_cosim cosim.launch.py num_robots:=$N period_ms:=$PERIOD duration_s:=$DUR \
     port:=$PORT csv_path:="$CSV" > /tmp/cosim-ros.log 2>&1 &
ROS_PID=$!
sleep 4   # let the bridge start listening and the robots publish a first pose

cd /opt/ns-3-dev
./build/contrib/sidewalk/examples/ns3.42-sidewalk-robots-optimized \
    --numRobots=$N --msgPeriod=$PERIOD --cosimPort=$PORT --enableSensing=1 \
    --simTag=cosim --outputDir=/tmp/ "$@" > /tmp/cosim-ns3.log 2>&1 || echo "ns-3 exited with $?"
wait $ROS_PID || true
grep -E "sidewalk_bridge|robot_[0-9]+\]" /tmp/cosim-ros.log | grep -v "process started\|process has finished" | tail -8
echo "deliveries CSV: $CSV ($(($(wc -l < "$CSV") - 1)) rows)"
