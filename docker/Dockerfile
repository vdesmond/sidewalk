# sidewalk dev image: CTTC ns-3 fork + 5G-LENA NR V2X (sidelink Mode 2) + NIST ns3-cosim, pinned.
# Same steps as docker/setup.sh (which is what was verified end-to-end on arm64).
# Build needs ~9 GB RAM at -j4; a -j10 build OOM-killed a 12 GB VM.
FROM ubuntu:22.04
COPY setup.sh /spike/setup.sh
RUN bash /spike/setup.sh
WORKDIR /opt/ns-3-dev
