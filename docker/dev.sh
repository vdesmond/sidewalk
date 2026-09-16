#!/usr/bin/env bash
# (Re)create the persistent dev container. Source+build live on the docker volume
# `slv2x-opt` (/opt/ns-3-dev); this repo is bind-mounted at /sidewalk.
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
docker rm -f slv2x 2>/dev/null || true
docker run -d --name slv2x --memory=9g \
  -v slv2x-opt:/opt -v "$REPO":/sidewalk -w /opt/ns-3-dev \
  ubuntu:22.04 sleep infinity
# the volume only holds /opt: (re)install toolchain + runtime libs in the fresh container
docker exec slv2x bash -c 'export DEBIAN_FRONTEND=noninteractive; apt-get update -qq && apt-get install -y -qq --no-install-recommends \
  g++ cmake ninja-build make git python3 pkg-config ca-certificates \
  libc6-dev libeigen3-dev sqlite3 libsqlite3-dev libgsl-dev libxml2-dev > /dev/null'
# first time only: clone + build into the volume
docker exec slv2x test -x /opt/ns-3-dev/ns3 || docker exec slv2x bash /sidewalk/docker/setup.sh
echo "ready: docker exec -it slv2x bash"
