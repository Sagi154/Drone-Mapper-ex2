---
name: write-component-test
description: Creates GTest/GMock component or integration tests for Drone-Mapper-ex2 with correct filter prefixes and isolation. Use when adding tests under tests/components/ or tests/integration/, or when fixing test structure for grading.
---

# Write Component Test

## File layout

- `tests/components/test_<component>.cpp` — component tests
- `tests/integration/test_<flow>.cpp` — integration tests

## Suite naming (required for filters)

GTest filter uses suite name prefix. Match assignment exactly:

```cpp
TEST(SimulationManagerTest, ExpandsCartesianProduct) { ... }  // SimulationManager.*
TEST(DroneControlTest, ScanAfterMovement) { ... }              // DroneControl.*
TEST(IntegrationTest, FullFlowWithRealAlgorithm) { ... }       // Integration.*
```

## Required suites

`SimulationManager`, `SimulationRun`, `MissionControl`, `DroneControl`, `MappingAlgorithm`, `MockLidar`, `MapsComparison`, `Integration`

`SimulationRun` tests must cover `MockGPS` and `MockMovement`.

## Isolation (grading)

Per assignment doc — instructors inject bugs and run your suite per component:

- Mock only **direct** dependencies via GMock interfaces.
- Do not include unrelated component internals in assertions.
- Bug in component X should be caught by X's tests (≥60% per assignment; Review Guideline cites >50% full / >25% partial).
- Integration tests should catch ≥20% of injected bugs (assignment).
- A destructive bug may fail multiple suites — still isolate where possible.

## MockLidar

Skeleton already implements `MockLidar`; write tests only. Example bug: rays reach only 2/3 of `z_max` — test obstacles at the beam's far end and other ray-boundary cases.

## Coverage checklist

- [ ] Happy path
- [ ] Boundary / invalid input
- [ ] Error returns and logging hooks
- [ ] Missing file paths (b06)

## Map format for test fixtures

All `.npy` test maps must follow the same format as `data_maps/benchmark_map.npy`:

- dtype `uint8`, 3D C-order array `(nx, ny, nz)`
- Create with: `np.zeros((nx, ny, nz), dtype=np.uint8)`, set occupied voxels to `1`
- See `docs/map3d_impl_contract.md` for world↔voxel mapping formula

## Integration tests (min 2)

1. All components + **real** `MappingAlgorithmImpl`
2. All components + **mock** algorithm (GMock `IMappingAlgorithm`)
3. Full flow with instructor maps in `tests/data/instructor/` — assert `mission_score >= 90`
4. Optional legacy benchmark: `benchmark_map.npy` via `tests/data/configs/sim_benchmark.yaml` (component-style smoke; not a substitute for instructor scenarios)

Instructor integration maps: `scenario_house.npy`, `scenario_small.npy`, `scenario_big.npy` under `tests/data/instructor/map/`.

- Focused compositions: `tests/data/instructor/compositions/composition_{small_room,big_room,house_lower}.yaml`
- Path helpers: `test_support::instructorInputsDir()`, `loadInstructorFocusedComposition()` in `tests/support/ConfigFixtures.hpp`
- Tests: `Integration.Instructor*` in `tests/integration/test_integration_full_flow.cpp`

Each integration test must finish within **1 minute** — with and without injected bugs.

## Run

```bash
cmake --build --preset default
./build/drone_mapper_simulation_test --gtest_filter=DroneControl.*
```

## Anti-patterns

- Testing through `main()` for component tests
- Shared global state between test cases
- Exposing private members via `friend` for tests — prefer interfaces or test doubles
