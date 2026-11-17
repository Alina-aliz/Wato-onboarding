#!/usr/bin/env bash
set -euo pipefail
test_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
packages="${1:-$test_dir/../src/robot}"
test_binary="$(mktemp /tmp/wato-navigation-tests.XXXXXX)"
trap 'rm -f "$test_binary"' EXIT
"${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra -Werror -pedantic \
  -I "$packages/navigation_common/include" \
  -I "$packages/costmap/include" -I "$packages/map_memory/include" \
  -I "$packages/planner/include" -I "$packages/control/include" \
  "$test_dir/core_tests.cpp" \
  "$packages/costmap/src/costmap_core.cpp" \
  "$packages/map_memory/src/map_memory_core.cpp" \
  "$packages/planner/src/planner_core.cpp" \
  "$packages/control/src/control_core.cpp" -o "$test_binary"
"$test_binary"
