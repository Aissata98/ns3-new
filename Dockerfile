# Version-pinned dependencies, exact hash-checked V6 study source.
# Not claimed bit-reproducible: apt package versions and the upstream ns-3
# download are not content-pinned. Do not confuse the historical binary hash
# with the newly generated BUILD_REPRODUCED.json.
FROM ubuntu@sha256:4fbb8e6a8395de5a7550b33509421a2bafbc0aab6c06ba2cef9ebffbc7092d90
ENV DEBIAN_FRONTEND=noninteractive
ENV NS3_DIR=/opt/ns-allinone-3.40/ns-3.40
RUN apt-get update && apt-get install -y --no-install-recommends \
    g++ gcc cmake ninja-build make python3 python3-dev git ca-certificates \
    wget tar bzip2 libsqlite3-dev libgsl-dev \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt
RUN wget -q https://www.nsnam.org/releases/ns-allinone-3.40.tar.bz2 \
    && tar jxf ns-allinone-3.40.tar.bz2 \
    && rm ns-allinone-3.40.tar.bz2
RUN git clone https://github.com/rmartin5/aqua-sim-ng.git ${NS3_DIR}/src/aqua-sim-ng \
    && git -C ${NS3_DIR}/src/aqua-sim-ng checkout 0fbc166b8b12cf711f14379cb159393d8061e3a6
COPY MODEL_MANIFEST.json replay.py /artifact/
COPY model/ /artifact/model/
RUN python3 /artifact/replay.py verify-model
COPY model/ ${NS3_DIR}/scratch/
RUN useradd --uid 10001 --create-home ns3builder \
    && chown -R ns3builder:ns3builder ${NS3_DIR} /artifact
USER ns3builder
ENV USER=ns3builder
WORKDIR ${NS3_DIR}
RUN ./ns3 configure --build-profile=optimized --enable-examples --disable-python --disable-werror \
    && ./ns3 build -j 2 edc-reviewed test_closure_phy
RUN python3 /artifact/replay.py _record-build --output /artifact/BUILD_REPRODUCED.json
WORKDIR /output
ENTRYPOINT ["python3", "/artifact/replay.py"]
CMD ["--help"]
