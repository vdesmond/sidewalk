# Ultra-Low Latency Sidelink for Cooperative Robotics

This project is an end-to-end simulation of a robot swarm exchanging high-frequency state messages over **5G-NR Sidelink Mode 2 (PC5)**. 

Instead of relying on a central base station (gNB), the robots use distributed sensing and semi-persistent scheduling (SPS) to autonomously select radio resources. The goal is to guarantee ultra-low latency and high reliability for cooperative robotics (e.g., collision avoidance, swarm formation) in shared spectrum.

This repository features a  hybrid co-simulation using the following tools:
- **ns-3 / 5G-LENA** models the 5G-NR physical and MAC layers, including a custom latency-aware Mode 2 MAC scheduler (`nr-sl-ue-mac-scheduler-earliest`).
- **ROS 2 (Humble)** drives the robot mobility (random waypoint) and application logic.
- The two are bridged via TCP lock-step (`ns3-cosim`), meaning the ROS 2 network sees real, accurate radio latency on its topics.

## Architecture

```text
ROS 2 (Humble)                                        ns-3 / 5G-LENA
robot_node × N ──/pose──▶ bridge_node ──TCP──▶ RobotGateway (ns3::Gateway)
   (mobility,                 │  every step_ms:               │
    state messages)           │  "sec nsec [x y z send]×N"    │
                              │◀─ "[src:latency_us;…]×N" ─────┘
robot_node ◀──/neighbors──────┘
           (received states)
```

Here, the ROS 2 bridge advances simulated time by `step_ms` per tick, ns-3 processes the step, computes the complex radio propagation/interference, and replies with the successful packet deliveries.

## Repository Layout

This repo is structured as an ns-3 **contrib module**.
- `model/`: The core C++ ns-3 models.
  - `nr-sl-ue-mac-scheduler-earliest`: Latency-aware Mode 2 resource selection scheduler.
  - `robot-gateway`: The TCP gateway synchronizing ns-3 with ROS 2.
  - `sensing-trace-sink`: Hooks into the MAC layer to export 3GPP TS 38.214 §8.1.4 sensing metrics.
- `examples/sidewalk-robots.cc`: The main ns-3 simulation script orchestrating the UE configuration, mobility, and traffic.
- `ros2/sidewalk_cosim/`: The ROS 2 package containing the `robot_node` and `bridge_node`.
- `docker/`: Container definitions to build ns-3 and ROS 2 Humble without host pollution.
- `scripts/`: Batch execution scripts for parameter sweeps and co-simulation runs.
- `analysis/`: Python scripts (pandas/matplotlib) to parse the SQLite/CSV traces and generate KPIs (PRR, latency CDFs).

## How to Run

Everything runs inside a Docker dev container to avoid polluting your host with ns-3 and ROS 2 dependencies.

```bash
# build the persistent dev container (installs ns-3, 5G-LENA, and ROS 2)
bash docker/dev.sh

# drop into the container
docker exec -it slv2x bash

# run the standalone ns-3 density sweep
bash /sidewalk/scripts/sweep.sh robots-density

# or run the full ROS 2 + ns-3 co-simulation
bash /sidewalk/scripts/cosim.sh 10 100 20  # 10 robots, 100ms msgs, 20s duration
```

## Visualization

You can generate a beautiful MP4/GIF animation of the robot swarm communicating directly from the SQLite packet traces! The script reconstructs the mobility and highlights successful 5G Sidelink message deliveries in real-time.

```bash
# Aasuming you've already generated a results database:
.venv/bin/python analysis/animate.py results/robots-density/nodes=10_scheme=1_run=1-sidewalk-robots.db
```
*(This outputs an MP4 video showing the 50x50m area with nodes flashing on message reception).*
