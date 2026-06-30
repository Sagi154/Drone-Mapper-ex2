# Drone-Mapper-ex2 — Agent Guide

TAU Advanced Topics in Programming, Assignment 2. Course skeleton is merged at repo root (`include/`, `src/`). Stubs compile; implementation in progress — see `workplan.md`.

## Start here

1. Read `.cursor/rules/project-context.mdc` (always applies).
2. Before git branches, commits, or PRs: `git-workflow.mdc`.
3. Before editing C++: `adv-cpp-standards.mdc`, `mp-units-strong-types.mdc`, `ex1-anti-patterns.mdc`.
4. Before touching interfaces: `frozen-interfaces.mdc`.
5. Before tests: `testing-requirements.mdc`.

## Skills (invoke by name)

| Skill | Use when |
|-------|----------|
| `implement-ex2-component` | Implementing or wiring a skeleton component |
| `port-from-ex1` | Reusing algorithm ideas from `Drone-Mapper/` ex1 |
| `write-component-test` | Adding GTest/GMock tests |
| `pre-submission-review` | Final checklist before commit/submission |
| `debug-simulation-failure` | Scenario errors, timeouts, score -1 |
| `maps-comparison-utility` | `MapsComparison` class or `maps_comparison` exe |

## Key docs

| Path | Purpose |
|------|---------|
| `docs/assignment2-checklist.md` | Mandatory requirements (condensed) |
| `docs/review-error-codes.md` | Grading error codes (e*, b*) |
| `docs/ex1-mistakes.md` | Ex1 deductions — do not repeat |
| `docs/ex1-to-ex2-porting.md` | Component mapping ex1 → ex2 |
| `refactor_by_component.md` | Full porting table |
| `workplan.md` | Team execution plan |
| `readme.txt` | Build/run + **agreed output layout** (`run_NNNN`, YAML schema) |
| `docs/map3d_impl_contract.md` | `.npy` format, world↔voxel mapping, reference maps |
| `context/` | Assignment docx (`Advanced Topics TAU 2026B - Assignment 2 (1).docx`) and review guidelines |

## Test maps

Hidden-map inputs are `uint8` `.npy` (C-order). On read, `0` = Empty and any value `>= 1` = Occupied (instructor maps may store `2`, `18`, `45`, etc. in occupied cells — all count as Occupied per course staff). Mutable output maps use `int8` with the full `VoxelOccupancy` enum (`-3` … `1`). See `docs/map3d_impl_contract.md`.

### Instructor integration maps (grading scenarios)

Vendored into `tests/data/instructor/` — use these for integration tests and CI.

| Map | Shape | Used by |
|-----|-------|---------|
| `scenario_house.npy` | `(29, 30, 31)` | `house_simulation.yaml` |
| `scenario_small.npy` | `(20, 20, 20)` | `small_simulation_room.yaml`, `small_simulation_out.yaml` |
| `scenario_big.npy` | `(30, 30, 30)` | `large_simulation_room.yaml`, `large_simulation_out.yaml` |

All instructor sim configs use `map_resolution_cm: 10`. House sim uses `map_axes_offset.height_offset: 150`.

- Full composition: `tests/data/instructor/sim_compose.yaml` (6 aligned pairs × 2 drones × 2 lidars = 24 runs)
- Focused compositions (integration/CI): `tests/data/instructor/compositions/composition_{small_room,big_room,house_lower}.yaml`
- Path helpers: `test_support::instructorInputsDir()`, `loadInstructorFocusedComposition()` in `tests/support/ConfigFixtures.hpp`

Run locally:

```bash
./build/drone_mapper_simulation tests/data/instructor/compositions/composition_small_room.yaml /tmp/instructor_small
grep mission_score /tmp/instructor_small/simulation_output.yaml
```

CI: the `docker-build-test` job runs focused compositions after unit tests; scores are printed in the **Check instructor composition mission scores** step log.

### Dev/unit-test maps (`data_maps/`)

Small golden maps (`single_voxel_*.npy`, etc.) and `benchmark_map.npy` for component tests. `benchmark_map.npy` is byte-identical to `scenario_house.npy`. Do **not** substitute these for instructor scenarios in integration grading.

## Build and run

```bash
cmake --preset default
cmake --build --preset default
./build/drone_mapper_simulation [<simulation.yaml>] [<output_path>]
./build/drone_mapper_simulation_test
./build/maps_comparison <origin_map> <target_map> [comparison_config=<path>]
```

## Grading (Assignment 2 + Review Guideline)

Submissions are graded by **bug injection** against your test suite. Thresholds from the assignment doc (`context/Advanced Topics TAU 2026B - Assignment 2 (1).docx`); the Review Guideline (`context/Exercise 2 Review Guideline.docx`) states lower bars for reference.

- **≥60%** of injected bugs caught per component test suite (assignment); Review Guideline cites >50% full / >25% partial.
- **≥20%** of injected bugs caught by integration tests (assignment).
- Isolate tests per component where possible; some bugs may legitimately fail multiple suites.
- Integration tests use instructor-provided config/map files; must finish within **1 minute** (with and without bugs).
- Manual code review is lighter than ex1 — comments in most cases; only extremely poor practices affect the grade.
- **Submission deadline:** July 5, 2026, 23:30.

## Hard rules

- **Never** change skeleton interfaces marked "Do not change".
- **Never** reimplement `MockLidar` or `ScanResultToVoxels`.
- **Never** use `exit()` / `abort()`; return from `main` gracefully.
- **Never** unwrap mp-units to `double` for math; use quantity arithmetic.
- Ex1 (`../Drone-Mapper/`) is **logic reference only** — on conflict, Assignment 2 docx wins over Review Guideline.
- Log all errors **immediately** to the per-run error log; never defer (do not port ex1 `flushIfNeeded`).
- **Rubric codes** (`e05`, `b04`, …) ≠ **runtime codes** (`ErrorRef::code` in YAML) — see `docs/review-error-codes.md`.
- One failed scenario → `score: -1` and continue; do not abort the whole composition unless startup is impossible.
- Keep `docs/HLD.md` aligned with code (graded).

## Test filters (required names)

`SimulationManager.*`, `SimulationRun.*`, `MissionControl.*`, `DroneControl.*`, `MappingAlgorithm.*`, `MockLidar.*`, `MapsComparison.*`, `Integration.*`
