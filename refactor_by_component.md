# Assignment 2 — Refactor Plan by Component

Maps each ex2 component to its skeleton location, ex1 reference, and implementation status.  
**Ex1 is logic reference only** — see `docs/ex1-mistakes.md` for patterns that caused grade deductions.

## Legend

| Column | Meaning |
|--------|---------|
| **Reuse %** | Estimated ex1 *logic* carry-over (not copy-paste) |
| **Status** | `Provided` = use skeleton as-is · `Stub` = replace implementation · `New` = create from scratch |
| **Done when** | Acceptance criteria for the component |

## Repo layout (skeleton merged)

| Path | Role |
|------|------|
| `include/drone_mapper/*.h` | Public interfaces and component headers |
| `include/drone_mapper/types/*.h` | Config and result data types |
| `src/*.cpp` | Component implementations |
| `src/drone_mapper_simulation_main.cpp` | Simulator entry point |
| `src/maps_comparison_main.cpp` | Standalone comparison utility |
| `data_maps/*.npy` | Tiny example maps for development |
| `cpp_yaml_example/` | yaml-cpp usage reference |
| `tests/components/`, `tests/integration/` | **To create** — required GTest suites |

## Component mapping

| Ex2 component | Skeleton files | Ex1 reference | Reuse % | Status | Refactor notes | Done when |
|---------------|----------------|---------------|--------:|--------|----------------|-----------|
| `MockLidar` | `MockLidar.h/cpp` | `src/simulation/LidarMock.*` | — | **Provided** | Course implementation — integrate, do not redesign. Ensure `scan()` is `const` (e07). | Tests pass under `MockLidar.*` |
| `ScanResultToVoxels` | `ScanResultToVoxels.h/cpp` | — | — | **Provided** | Applies voxels to output map — call from `DroneControlImpl`, do not redesign. | Wired in drone step pipeline |
| `MockGPS` | `MockGPS.h/cpp` | `PositionMock.*` | 95% | Stub | Port pose/heading state; constructor takes `gps_resolution`. | Position/heading consistent with movement |
| `MockMovement` | `MockMovement.h/cpp` | `MovementMock.*`, `CollisionDetector.*` | 85% | Stub | Rotate/advance/elevate + collision against hidden map. | Limits enforced; updates `MockGPS` |
| `Map3DImpl` | `Map3DImpl.h/cpp` | `BuildingMap.*` | 40% | Stub | `.npy` via TinyNPY; `MapConfig` (bounds, offset, resolution); `atVoxel`, `set`, `isInBounds`, `save`. **Not** `SimulationState`. | Round-trip .npy; bounds correct |
| `MappingAlgorithmImpl` | `MappingAlgorithmImpl.h/cpp` | `DroneAlgorithm.*`, `ExplorationFrontier.*` | 70% | Stub | `tick()` → `nextStep(state, latest_scan)`. **Do not port** `ScanProbing`, `TickSnapshot`, `ScanOptions` (e16n). | Valid commands; mission completes |
| `DroneControlImpl` | `DroneControlImpl.h/cpp` | ex1 runtime loop | 50% | Stub | `step()`: movement → scan → `ScanResultToVoxels` → `set`. First `nextStep` gets `nullptr` scan. | One step executes full pipeline |
| `MissionControlImpl` | `MissionControlImpl.h/cpp` | ex1 `main.cpp` loop | 40% | Stub | Loop `drone_control.step()` until done / `max_steps` / error; save output map. | Correct `MissionRunResult` |
| `MapsComparison` | `MapsComparison.h/cpp`, `maps_comparison_main.cpp` | `Scorer.*` | 70% | Stub | Scoring math from ex1; ex2 `IMap3D` + optional comparison YAML. CLI: stdout score only. | 100 / ~100 / ~0 behavior |
| `SimulationRunImpl` | `SimulationRunImpl.h/cpp` | ex1 result assembly | 30% | Stub | Run mission → compare maps → assemble `SimulationResult`. | Complete result per run |
| `SimulationRunFactoryImpl` | `SimulationRunFactoryImpl.h/cpp` | ex1 DI in `main` | 30% | Stub | Wire object graph per scenario tuple; load hidden `.npy`. | Runnable `ISimulationRun` per combo |
| `SimulationManager` | `SimulationManager.h/cpp` | (new) | 20% | Stub | Cartesian product; build `SimulationManagerReport`; write `simulation_output.yaml`. | All composition runs aggregated |
| `drone_mapper_simulation_main` | `drone_mapper_simulation_main.cpp` | ex1 `main.cpp` | 25% | Stub | Load composition YAML; path rules; delegate to manager; write `output_results/`. | CLI matches assignment |
| YAML parsing | *to create* e.g. `src/io/` | `DroneConfigParser`, `MissionConfigParser` | 50% | **New** | Validation spirit from ex1; YAML format; immediate error logging. Link `yaml-cpp` in CMake. | All config types + composition parse |
| Error logging | *to create* e.g. `src/io/` | ex1 `ErrorLogger` | 20% | **New** | **No** `lines()` getter (e22m). Structured `ErrorRef`; flush immediately. | Errors logged at occurrence |
| Test suites | `tests/` | ex1 `tests/` | 35% | **New** | Required filters; component isolation for bug-injection grading. | All filters pass |

## Do not reimplement

- `MockLidar` — provided
- `ScanResultToVoxels` — provided

## Do not port from ex1

`ScanProbing`, `ScanOptions`, `TickSnapshot`, `SimulationState`, `ErrorLogger::lines()`, ex1 `main` loop, txt parsers, per-function `namespace su = mp_units::...`.

## Recommended implementation order

1. **Foundation:** `Map3DImpl`, YAML parsing, error logger, CMake + `yaml-cpp` link
2. **Runtime:** `MockGPS`, `MockMovement`, `MappingAlgorithmImpl`, `DroneControlImpl`, `MissionControlImpl`
3. **Orchestration:** `SimulationRunFactoryImpl`, `SimulationRunImpl`, `SimulationManager`, `drone_mapper_simulation_main`
4. **Scoring:** `MapsComparison` + `maps_comparison` exe
5. **Tests:** `tests/components/` then `tests/integration/`; wire `drone_mapper_simulation_test` in CMake
6. **After Gate B passes:** algorithm efficiency (see `workplan.md` Phase 5)

## Completion gates

| Gate | Criteria |
|------|----------|
| **A — Build** | All targets compile; one stub run executes without crash |
| **B — Compliance** | Outputs, CLI, test filters, error handling match assignment |
| **C — Quality** | Tests stable; no ex1 anti-patterns; small maps &lt; ~10s (avoid b05s) |
| **D — Polish** | Algorithm efficiency, HLD PDF, `readme.txt`, submission package |

## Agent workflow

Use Cursor skill `implement-ex2-component` per row; `port-from-ex1` for algorithm/collision/scoring ideas only. Run `pre-submission-review` before each gate.
