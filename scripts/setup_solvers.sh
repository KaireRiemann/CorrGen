#!/usr/bin/env bash
set -euo pipefail
# Run inside ros1_noetic. Dependencies stay in the workspace, not system Python.
workspace="${1:-/root/ws/corr_gen}"
mkdir -p "$workspace/.deps"
if [ ! -f "$workspace/.deps/proxsuite/cmeel.prefix/include/proxsuite/proxqp/dense/dense.hpp" ]; then
  python3 -m pip install --target "$workspace/.deps/proxsuite" --no-deps proxsuite==0.6.3
fi
if [ ! -d "$workspace/.deps/HiGHS/.git" ]; then
  git clone --depth 1 --branch v1.10.0 https://github.com/ERGO-Code/HiGHS.git "$workspace/.deps/HiGHS"
fi
test "$(git -C "$workspace/.deps/HiGHS" rev-parse HEAD)" = fd8665394edfd096c4f847c4a6fbc187364ef474
cmake -S "$workspace/.deps/HiGHS" -B "$workspace/.deps/highs-build" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$workspace/.deps/highs" -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DFAST_BUILD=ON
cmake --build "$workspace/.deps/highs-build" -j2
cmake --install "$workspace/.deps/highs-build"
