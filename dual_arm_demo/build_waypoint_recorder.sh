#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$ROOT/.." && pwd)"
SDK_ROOT="$REPO_ROOT/jaka_ros2/src/jaka_driver"

g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic \
  -I"$SDK_ROOT/include/jaka_driver" \
  "$ROOT/waypoint_recorder.cpp" \
  -L"$SDK_ROOT/lib" -ljakaAPI \
  -Wl,-rpath,"$SDK_ROOT/lib" \
  -pthread \
  -o "$ROOT/waypoint_recorder"

g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic \
  "$ROOT/waypoint_recorder_core_test.cpp" \
  -o "$ROOT/waypoint_recorder_core_test"

echo "Built: $ROOT/waypoint_recorder"
echo "Built: $ROOT/waypoint_recorder_core_test"
