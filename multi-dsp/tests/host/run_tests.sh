#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Louis Brooks

# Builds and runs the host-side tests. Needs only g++ (C++17). Python + Pillow are needed for screenshots.
#   ./run_tests.sh            run all tests
#   ./run_tests.sh screens    also regenerate docs/img/*.png
set -euo pipefail
cd "$(dirname "$0")"
OUT=build; mkdir -p "$OUT"
CXX=${CXX:-g++}
FLAGS="-std=gnu++17 -O2 -w -Istubs -Istubs/pico"
for t in test_cpu_stats test_ringo test_quirks test_encoder test_knobs; do
  echo "=== $t ==="
  $CXX $FLAGS $t.cpp -o "$OUT/$t"
  "$OUT/$t"
  echo
done
if [ "${1:-}" = "screens" ]; then
  echo "=== render_screens ==="
  $CXX $FLAGS render_screens.cpp -o "$OUT/render_screens"
  (cd "$OUT" && ./render_screens)
  python3 -I topng.py "$OUT" ../../docs/img
fi
