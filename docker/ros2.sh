#!/usr/bin/env bash
# Install ROS 2 Humble (ros-base) in the dev container and build ros2/sidewalk_cosim.
# Idempotent; run by docker/dev.sh.
set -eo pipefail
export DEBIAN_FRONTEND=noninteractive
if [ ! -f /opt/ros/humble/setup.bash ]; then
  apt-get update -qq && apt-get install -y -qq --no-install-recommends curl gnupg lsb-release locales > /dev/null
  locale-gen en_US.UTF-8 > /dev/null
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list
  apt-get update -qq && apt-get install -y -qq --no-install-recommends \
    ros-humble-ros-base ros-humble-geometry-msgs ros-dev-tools python3-colcon-common-extensions > /dev/null
fi
source /opt/ros/humble/setup.bash
mkdir -p /opt/ros2_ws/src
ln -sfn /sidewalk/ros2/sidewalk_cosim /opt/ros2_ws/src/sidewalk_cosim
cd /opt/ros2_ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -1
