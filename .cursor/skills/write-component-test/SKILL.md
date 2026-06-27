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

Per Review Guideline — instructors inject bugs and run your suite per component:

- Mock only **direct** dependencies via GMock interfaces.
- Do not include unrelated component internals in assertions.
- Bug in component X should be caught by X's tests (>50% for full marks, >25% for partial).
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
3. Full flow with `benchmark_map.npy` (ClassicCube-2, `29×30×31`) — assert `mission_score` ≥ 90 (instructor-provided scenario; score should approach 100)
4. Ex1-ported maps `scenario_{3,4,5}_map.npy` — use `scenarios/composition_scenarioN.yaml`; layout from ex1 grading scenarios, converted to ex2 format at 10 cm/voxel

Instructors provide config and map files for integration grading. Each integration test must finish within **1 minute** — with and without injected bugs. Use `tests/data/configs/sim_benchmark.yaml` and `test_support::benchmarkMapPath()` for the benchmark; `scenarios/composition_scenarioN.yaml` for ex1-ported maps.

## Run

```bash
cmake --build --preset default
./build/drone_mapper_simulation_test --gtest_filter=DroneControl.*
```

## Anti-patterns

- Testing through `main()` for component tests
- Shared global state between test cases
- Exposing private members via `friend` for tests — prefer interfaces or test doubles
