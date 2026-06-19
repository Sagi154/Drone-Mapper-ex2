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

- Mock only **direct** dependencies via GMock interfaces.
- Do not include unrelated component internals in assertions.
- Bug in component X should fail only X's tests (≥60% detection target).

## Coverage checklist

- [ ] Happy path
- [ ] Boundary / invalid input
- [ ] Error returns and logging hooks
- [ ] Missing file paths (b06)

## Integration tests (min 2)

1. All components + **real** `MappingAlgorithmImpl`
2. All components + **mock** algorithm (GMock `IMappingAlgorithm`)

## Run

```bash
cmake --build --preset default
./build/drone_mapper_simulation_test --gtest_filter=DroneControl.*
```

## Anti-patterns

- Testing through `main()` for component tests
- Shared global state between test cases
- Exposing private members via `friend` for tests — prefer interfaces or test doubles
