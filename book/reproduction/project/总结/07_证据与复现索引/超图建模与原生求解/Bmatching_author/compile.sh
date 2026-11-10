#!/usr/bin/env bash
set -euo pipefail

BUILD_DIR="build"
BUILD_TYPE="${1:-Release}"

cmake -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
  -DBUILD_TESTING=OFF

cmake --build "$BUILD_DIR" -j"$(nproc)" \
  --target app bmatching_cli runner fork_runner generate_experiment_config binary_to_textproto
