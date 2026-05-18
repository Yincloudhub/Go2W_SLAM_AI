#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
mkdir -p build
if [ -f build/CMakeCache.txt ]; then
  cached_source_dir="$(grep '^CMAKE_HOME_DIRECTORY:INTERNAL=' build/CMakeCache.txt | cut -d= -f2- || true)"
  current_source_dir="$(pwd)"
  if [ -n "$cached_source_dir" ] && [ "$cached_source_dir" != "$current_source_dir" ]; then
    echo "Detected stale CMake cache:"
    echo "  cached source:  $cached_source_dir"
    echo "  current source: $current_source_dir"
    echo "Recreating build configuration."
    rm -rf build/CMakeCache.txt build/CMakeFiles
  fi
fi
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release "$@"
make -j"$(nproc)"
