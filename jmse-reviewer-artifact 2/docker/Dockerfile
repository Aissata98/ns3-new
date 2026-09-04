# ns-3.40 + Aqua-Sim NG build image for the EDC underwater-pipeline simulator.
#
# This bakes the heavy, stable layer (ns-3.40 core + aqua-sim-ng, fully built).
# edc.cc is NOT baked in — it is mounted at runtime into scratch/ so edits
# recompile incrementally (seconds) instead of rebuilding the image.
#
# Recipe follows the nsnam.com "Build Aqua-Sim NG in NS-3.40 on Ubuntu 24.04"
# guide + the rmartin5/aqua-sim-ng fork (the one compatible with ns-3.40).
# Base image pinned by digest; apt repositories and downloaded archives mean the
# recipe is version-pinned but not, by itself, a byte-reproducible OCI image.
FROM ubuntu@sha256:4fbb8e6a8395de5a7550b33509421a2bafbc0aab6c06ba2cef9ebffbc7092d90

ENV DEBIAN_FRONTEND=noninteractive
ENV NS3_DIR=/opt/ns-allinone-3.40/ns-3.40

# Build toolchain + ns-3 runtime deps. AquaSim needs nothing beyond core ns-3.
RUN apt-get update && apt-get install -y --no-install-recommends \
        g++ gcc cmake ninja-build make \
        python3 python3-dev \
        git ca-certificates wget tar bzip2 \
        libsqlite3-dev libgsl-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt

# ns-3.40 source (all-in-one tarball).
RUN wget -q -c https://www.nsnam.org/releases/ns-allinone-3.40.tar.bz2 \
    && tar jxf ns-allinone-3.40.tar.bz2 \
    && rm ns-allinone-3.40.tar.bz2

# Aqua-Sim NG as an ns-3 module under src/ (the fork that builds on 3.40).
# Pinned to an exact commit so the build is reproducible even if the fork moves.
ARG AQUASIM_COMMIT=0fbc166b8b12cf711f14379cb159393d8061e3a6
RUN git clone https://github.com/rmartin5/aqua-sim-ng.git \
        ${NS3_DIR}/src/aqua-sim-ng \
    && git -C ${NS3_DIR}/src/aqua-sim-ng checkout ${AQUASIM_COMMIT}

WORKDIR ${NS3_DIR}

# Configure + build core ns-3 + aqua-sim-ng once. edc.cc is added at runtime.
# --disable-werror keeps AquaSim's older-style warnings from breaking the build.
RUN ./ns3 configure --build-profile=optimized \
        --enable-examples --disable-python --disable-werror \
    && ./ns3 build

# Runtime entrypoint just drops into a shell at the ns-3 root; the wrapper
# script (dsim.sh) mounts scratch/edc.cc + run_edc_sweep.py and invokes ./ns3.
CMD ["/bin/bash"]
