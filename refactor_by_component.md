# Assignment 2 — Refactor Plan by Component

Maps each ex2 component to its skeleton location, ex1 reference, and implementation status.  
**Ex1 is logic reference only** — see `docs/ex1-mistakes.md` for patterns that caused grade deductions.  
**Schedule, overlap, and handoffs:** `workplan.md`.

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

Matches `workplan.md` phases and handoffs (H1–H5). **Dependency order ≠ one person working alone that week** — runtime (Phase 2) and orchestration (Phase 3) overlap from Day 4.

```
Day:  1    2    3    4    5    6    7    8    9
      |----- Foundation -----|
                        |-------- Runtime (B leads) --------|
                             |-------- Orchestration (A leads) --------|
                                                  |-- Tests --|
```

### 1. Foundation (Phase 1)

| Owner | Components |
|-------|------------|
| **A** | `Map3DImpl`, YAML parsing, error logger, CMake + `yaml-cpp` link, compile-only `SimulationRunFactoryImpl` stub |
| **B** | `MockGPS`, `MockMovement`, test harness (`tests/components/` + `drone_mapper_simulation_test` in CMake) |

**Gate (partial):** load `data_maps/*.npy`, parse hand-written YAML, log a test error — **H1** unblocks runtime work.

### 2. Runtime + scoring (Phase 2, Days 3–6)

| Owner | Components | Notes |
|-------|------------|-------|
| **B** (primary) | `MappingAlgorithmImpl` → `DroneControlImpl` → `MissionControlImpl` → `MapsComparison` + `maps_comparison` exe | Wire `ScanResultToVoxels` in drone step · port scoring ideas from ex1 `Scorer` |
| **B** | Component tests: `DroneControl.*`, `MissionControl.*`, `MappingAlgorithm.*`, `MapsComparison.*` | Start as each component lands |
| **A** (parallel) | Minimal single-scenario `SimulationRunFactoryImpl`, `SimulationRunImpl` skeleton, YAML/`.npy` test fixtures, `output_results/` layout draft | Starts Day 3 — do not wait for full runtime |

**Integrate (do not redesign):** `MockLidar`, `ScanResultToVoxels` when wiring `DroneControlImpl`.

**Handoffs:** **H2** — B shares `DroneControlImpl::step` deps when it compiles · **H3** — B's `runMission` plugs into A's `SimulationRunImpl` / factory stub.

**Gate (full):** one scenario end-to-end, output `.npy` written — **A + B** (not B alone).

### 3. Orchestration (Phase 3, Days 4–8 — overlaps Phase 2)

| Owner | Components | Notes |
|-------|------------|-------|
| **A** (primary) | Generalize `SimulationRunFactoryImpl`, complete `SimulationRunImpl`, `SimulationManager`, `drone_mapper_simulation_main`, missing-input handling (b06) | Expand Day 3 stub to full cartesian product |
| **B** (parallel) | Finish remaining runtime tests, help factory wiring at **H3**, `SimulationManager.*` / `SimulationRun.*` tests | Pair on DI graph per `docs/HLD.md` |

**Gate B:** `./drone_mapper_simulation` with small composition YAML → `simulation_output.yaml` + `output_results/`; continuable failure → score `-1`.

### 4. Tests & compliance (Phase 4, Days 7–9)

| Owner | Components |
|-------|------------|
| **B** | `tests/integration/` (`Integration.*`), GMock `IMappingAlgorithm`, `MockLidar.*` |
| **Both** | Bug-isolation review (unrelated suites stay green when one component broken), `docs/HLD.md` updates |
| **A** | `readme.txt` — build, run, output formats |

**Gate C:** all required `--gtest_filter` prefixes green; invalid config and missing files handled gracefully.

### 5. Efficiency & submission (Phase 5 — after Gate C only)

Algorithm tuning (avoid b05s), HLD PDF, final sanity runs — see `workplan.md` Phase 5 · **Gate D**.

## Completion gates

| Gate | Criteria |
|------|----------|
| **A — Build** | All targets compile; one stub run executes without crash |
| **B — Compliance** | Outputs, CLI, test filters, error handling match assignment |
| **C — Quality** | Tests stable; no ex1 anti-patterns; small maps &lt; ~10s (avoid b05s) |
| **D — Polish** | Algorithm efficiency, HLD PDF, `readme.txt`, submission package |

## Agent workflow

Use Cursor skill `implement-ex2-component` per row; `port-from-ex1` for algorithm/collision/scoring ideas only. Run `pre-submission-review` before each gate. Respect phase overlap and H1–H5 handoffs in `workplan.md` — owner column in the order section is primary implementer, not solo worker.
