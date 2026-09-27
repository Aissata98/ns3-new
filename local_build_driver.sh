#!/bin/bash
# Runs only inside the new bounded build container, never the campaign one.
set -euo pipefail
cd /artifact
python3 -c 'import replay; print(replay.check_cgroup())'
python3 replay.py verify-model
python3 local_build.py _check-dependencies --output /artifact/LOCAL_DEPENDENCY_CHECK.json
ns3_task_root=/opt/ns-allinone-3.40/ns-3.40
cp /artifact/model/* "$ns3_task_root/scratch/"
useradd --uid 10001 --create-home ns3builder
chown ns3builder:ns3builder "$ns3_task_root" "$ns3_task_root/.lock-ns3_linux_build"
chown -R ns3builder:ns3builder "$ns3_task_root/scratch" "$ns3_task_root/build" "$ns3_task_root/cmake-cache" /artifact
runuser -u ns3builder -- env USER=ns3builder bash -c '
  set -euo pipefail
  cd /opt/ns-allinone-3.40/ns-3.40
  git config --global --add safe.directory "$PWD/src/aqua-sim-ng"
  ./ns3 configure --build-profile=optimized --enable-examples --disable-python --disable-werror
  ./ns3 build -j 2 edc-reviewed test_closure_phy
  python3 /artifact/replay.py _record-build --output /artifact/BUILD_REPRODUCED.json
'
