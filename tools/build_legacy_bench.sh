#!/usr/bin/env bash
# Build the legacy comparison tools against the old sentry tree.
#   ./tools/build_legacy_bench.sh [legacy_root]
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

LEGACY="${1:-/home/lgnatius/Desktop/06_参考项目_auto-aim-master}"
CXX="${CXX:-g++}"
FLAGS=(-O2 -std=c++17 -DSENTRY -DAUTOAIM_LEGACY_ROOT="\"${LEGACY}\"")

mkdir -p build
# old detector-only benchmark
"$CXX" "${FLAGS[@]}" tools/legacy_detector_bench.cpp \
  "${LEGACY}/Armor/src/ArmorDetector.cpp" \
  -I"${LEGACY}/Armor/include" -I"${LEGACY}/Backbone/include" -I"${LEGACY}/tools" \
  -I/usr/include/eigen3 $(pkg-config --cflags --libs opencv4) -lyaml-cpp \
  -o build/legacy_detector_bench

# old full pipeline (detector -> solver -> EKF -> planner), judged by the new core
"$CXX" "${FLAGS[@]}" tools/legacy_pipeline.cpp \
  "${LEGACY}/Armor/src/ArmorDetector.cpp" \
  "${LEGACY}/Armor/src/ArmorSolver.cpp" \
  "${LEGACY}/Armor/src/ArmorTarget.cpp" \
  "${LEGACY}/Armor/src/ArmorEKF.cpp" \
  "${LEGACY}/Armor/src/ArmorPredictor.cpp" \
  "${LEGACY}/Armor/src/ArmorPlanner.cpp" \
  "${LEGACY}/tools/trajectory.cpp" \
  -Itools/legacy_include -I"${LEGACY}/Armor/include" -I"${LEGACY}/Backbone/include" \
  -I"${LEGACY}/tools" -I/usr/include/eigen3 -Iinclude \
  $(pkg-config --cflags --libs opencv4) -lyaml-cpp -Lbuild -lautoaim_core \
  -o build/legacy_pipeline

echo "built build/legacy_detector_bench build/legacy_pipeline"
