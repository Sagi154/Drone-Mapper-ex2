#!/usr/bin/env bash
# Build Drone-Mapper-ex2 from WSL. One-time host setup (requires sudo):
#   sudo apt-get update && sudo apt-get install -y g++ ninja-build
set -euo pipefail

export VCPKG_ROOT="${VCPKG_ROOT:-$HOME/vcpkg}"
export PATH="$HOME/.local/bin:$PATH"

if [[ ! -f "$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake" ]]; then
  echo "VCPKG_ROOT not found at $VCPKG_ROOT — clone vcpkg or set VCPKG_ROOT" >&2
  exit 1
fi

if ! command -v ninja >/dev/null 2>&1; then
  pip install -q ninja
fi

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repo_root"

cmake --preset default
cmake --build --preset default

echo "Build OK. Run integration tests:"
echo "  ./build/drone_mapper_integration_tests --gtest_filter=Integration.*"
