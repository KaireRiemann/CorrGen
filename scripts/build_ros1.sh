#!/usr/bin/env bash
set -euo pipefail
# Run on the host. Uses the existing /home/diffbot/ros1_ws -> /root/ws mount.
container="${1:-ros1_noetic}"
docker exec "$container" bash -lc '
  source /opt/ros/noetic/setup.bash
  cd /root/ws/corr_gen
  bash src/CorrGen/scripts/setup_solvers.sh /root/ws/corr_gen
  catkin_make -DCMAKE_BUILD_TYPE=Release -DCORRGEN_BUILD_ROS1=ON \
    -DCMAKE_PREFIX_PATH="/root/ws/corr_gen/.deps/highs;/opt/ros/noetic" \
    -DPROXQP_INCLUDE_DIR=/root/ws/corr_gen/.deps/proxsuite/cmeel.prefix/include -j2
  cd build/CorrGen
  ctest --output-on-failure
'
