# CI image: Ubuntu 24.04 C++ devcontainer stack + vcpkg (same family as the course skeleton).
# Reproduces Linux configure / build in one place for GitHub Actions and local checks.
#
# Tests are NOT run during `docker build` — they are executed via `docker run ... ctest`
# from the CI workflow (or locally) so each test suite surfaces as its own CI step and
# tests don't run twice (once at build time, once at run time).
#
# Usage (from repo root):
#   docker build -t drone-mapper-ex2-ci .
#   docker run --rm drone-mapper-ex2-ci ctest --test-dir build/ci --output-on-failure
#
# The base image sets VCPKG_ROOT (typically /usr/local/vcpkg). Dependencies come from vcpkg.json.
FROM mcr.microsoft.com/devcontainers/cpp:2-ubuntu24.04

ENV VCPKG_ROOT=/usr/local/vcpkg

WORKDIR /workspace

# Copy only what the CMake project needs (keeps build context small; see .dockerignore).
COPY vcpkg.json vcpkg-configuration.json CMakeLists.txt CMakePresets.json ./
COPY include ./include
COPY src ./src
COPY tests ./tests
COPY data_maps ./data_maps
COPY scenarios ./scenarios

ARG CMAKE_BUILD_PARALLEL_LEVEL=4
ENV CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL}

# Explicit toolchain path so CI does not depend on shell preset wiring inside the container.
# Build only — `ctest` is invoked as a separate CI step against this image.
RUN cmake -G Ninja -S . -B build/ci \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_TOOLCHAIN_FILE="${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" \
        -DDRONE_MAPPER_BUILD_YAML_EXAMPLE=OFF \
    && cmake --build build/ci
