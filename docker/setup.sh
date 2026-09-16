set -ex
export DEBIAN_FRONTEND=noninteractive
apt-get update && apt-get install -y --no-install-recommends \
  g++ cmake ninja-build make git python3 python3-pip pkg-config ca-certificates \
  libc6-dev libeigen3-dev sqlite3 libsqlite3-dev libgsl-dev libxml2-dev
cd /opt
[ -d ns-3-dev ] || git clone --depth 1 --branch ns-3-dev-v2x-v1.1 https://gitlab.com/cttc-lena/ns-3-dev.git
cd ns-3-dev/contrib
[ -d nr ] || git clone --depth 1 --branch v2x-1.1 https://gitlab.com/cttc-lena/nr.git
[ -d ns3-cosim ] || git clone --depth 1 https://github.com/usnistgov/ns3-cosim.git
cd ..
./ns3 configure -G Ninja --enable-examples --enable-tests --build-profile=optimized
./ns3 build -j4
echo BUILD_DONE
