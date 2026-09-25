#!/usr/bin/env bash
# Compile and run the host-side firmware tests.
#
# ⛔ WHY NOT PlatformIO's test runner. These tests exist so a control algorithm
# can be checked WITHOUT a board, a toolchain download, or a water test. Adding a
# platform dependency to reach them would defeat that. Anything under
# test/native/ must compile with a plain C++ compiler against src/ alone.
set -euo pipefail
cd "$(dirname "$0")/.."
CXX=${CXX:-g++}
FLAGS="-std=c++17 -O2 -Wall -Wextra -Werror -I src"
fail=0
for t in test/native/test_*.cpp; do
    name=$(basename "$t" .cpp)
    # Every src/ translation unit the test's subject needs, listed explicitly: a
    # glob would drag in the Arduino-dependent modules.
    case "$name" in
        test_allocator) srcs="src/control/allocator.cpp" ;;
        *)              srcs="" ;;
    esac
    out="$(mktemp -d)/$name"
    echo "--- $name"
    # shellcheck disable=SC2086
    $CXX $FLAGS "$t" $srcs -o "$out" -lm
    "$out" || fail=1
done
exit $fail
