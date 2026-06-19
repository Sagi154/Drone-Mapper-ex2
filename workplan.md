# Assignment 2 — Work Plan

**Deadline:** July 1, 2026, 23:30  
**Team:** 2 people · **Repo state:** course skeleton merged; all components are stubs except `MockLidar` and `ScanResultToVoxels`

## Goal

Ship a complete Assignment 2 submission: mandatory components, YAML/.npy I/O, tests with required GTest filters, graceful error handling, and a working (not necessarily optimal) mapping algorithm. Improve algorithm efficiency **only after** mandatory gates pass.

## Authority (when in doubt)

1. `context/` assignment docx  
2. Skeleton interfaces (`include/drone_mapper/`)  
3. `docs/review-error-codes.md` + Review Guideline  
4. Ex1 (`../Drone-Mapper/`) — algorithm ideas only · `docs/ex1-mistakes.md`

## Team split

| Owner | Primary responsibility |
|-------|----------------------|
| **Person A** | YAML parsing, `.npy` I/O, `Map3DImpl`, `SimulationManager`, output YAML, `output_results/` layout, error logger |
| **Person B** | `MockGPS`/`MockMovement`, drone/mission runtime, `MappingAlgorithmImpl`, `MapsComparison`, test harness |
| **Both** | Integration, Gate B/C review, Phase 5 efficiency, submission docs |

Use Cursor skills: `implement-ex2-component`, `port-from-ex1`, `write-component-test`, `pre-submission-review`.

---

## Phase 0 — Bootstrap (Day 0–1)

**Outcome:** Clean build from repo root; dev workflow agreed.

| Task | Owner | Deliverable |
|------|-------|-------------|
| Verify vcpkg + CMake preset build | Both | `cmake --preset default && cmake --build --preset default` green |
| Read `docs/assignment2-checklist.md`, `refactor_by_component.md` | Both | Shared understanding of gates |
| Run skeleton binaries once | Both | Note stub behavior baseline |
| Agree `output_results/` naming in `readme.txt` draft | A | Section in readme |

**Gate 0:** `drone_mapper_simulation` and `maps_comparison` executables build.

---

## Phase 1 — Foundation (Days 1–3)

**Outcome:** Maps load/save; configs parse; errors log immediately.

| Task | Owner | Files / notes |
|------|-------|---------------|
| `Map3DImpl` — TinyNPY load/save, `MapConfig`, `isInBounds` | A | `src/Map3DImpl.cpp` · golden tests on `data_maps/*.npy` |
| YAML parsers — drone, mission, lidar, simulation, composition | A | New `src/io/` (or similar) · link `yaml-cpp` in `CMakeLists.txt` |
| Error logger — structured codes, no exposed internals (e22) | A | New `src/io/` · immediate flush |
| `MockGPS` + `MockMovement` | B | Port collision ideas from ex1 · no ex1 class names |
| Component tests: `Map3DImpl` helpers, GPS, movement | B | Start `tests/components/` + CMake `drone_mapper_simulation_test` |

**Gate A:** Load a `data_maps/*.npy` map, parse a hand-written YAML, log a test error to file.

**Ex1 watchlist:** no `ErrorLogger::lines()`, no per-function mp-units aliases (e17s), strong types at parse boundary (e16).

---

## Phase 2 — Runtime loop (Days 3–5)

**Outcome:** Single mission runs: step loop, scan, voxels, save map.

| Task | Owner | Files / notes |
|------|-------|---------------|
| `MappingAlgorithmImpl::nextStep` — baseline frontier/BFS | B | Port **ideas** from `ExplorationFrontier` only · no `TickSnapshot` |
| `DroneControlImpl::step` — movement → scan → voxels | B | Wire `ScanResultToVoxels` · first step `latest_scan == nullptr` |
| `MissionControlImpl::runMission` — max_steps, status, save | B | `MissionRunResult` + errors |
| `MapsComparison` + `maps_comparison` exe | B | stdout score only · stderr on error |
| Component tests | B | `DroneControl.*`, `MissionControl.*`, `MappingAlgorithm.*`, `MapsComparison.*` |

**Gate A (full):** One manual factory wiring or minimal integration path completes a mission on a tiny map and writes output `.npy`.

**Performance note:** Profile small maps early — target ~10s, hard limit ~1 min (b05s).

---

## Phase 3 — Orchestration (Days 5–7)

**Outcome:** Full composition cartesian product; CLI; report files.

| Task | Owner | Files / notes |
|------|-------|---------------|
| `SimulationRunFactoryImpl` — wire DI graph per tuple | A | See `docs/HLD.md` factory sequence |
| `SimulationRunImpl` — mission + compare + `SimulationResult` | A | Score -1 on continuable errors |
| `SimulationManager` — expand product, aggregate report | A | `simulation_output.yaml` structure |
| `drone_mapper_simulation_main` — CLI path rules | A | Default `simulation.yaml`, relative/absolute paths |
| `output_results/` maps + per-run error logs | A | Document in `readme.txt` |
| Component tests | A/B | `SimulationManager.*`, `SimulationRun.*` (+ GPS/movement in Run suite) |
| Missing-input handling (b06) | A | Tests for missing yaml/map |

**Gate B:** Run `./drone_mapper_simulation` with a small composition YAML; get `simulation_output.yaml` + `output_results/`; failed scenario gets -1 and run continues.

---

## Phase 4 — Tests & compliance (Days 7–9)

**Outcome:** All required GTest filters pass; HLD matches code.

| Task | Owner | Deliverable |
|------|-------|-------------|
| Integration test — real algorithm | B | `tests/integration/` · filter `Integration.*` |
| Integration test — mock `IMappingAlgorithm` | B | GMock algorithm |
| `MockLidar` component tests | B | `MockLidar.*` filter |
| Bug-isolation review | Both | Unrelated suites don't fail when one component broken |
| Update `docs/HLD.md` if wiring changed | Both | Class + sequence diagrams (e14/e15) |
| `readme.txt` — build, run, output formats | A | Matches actual behavior |
| Run `pre-submission-review` skill checklist | Both | All boxes checked |

**Gate C:** `./drone_mapper_simulation_test` all green; all 8 component prefixes + `Integration.*` work; invalid config exits gracefully; missing files handled.

---

## Phase 5 — Efficiency & submission (Days 9–12, buffer)

**Start only after Gate C.**

| Task | Owner | Notes |
|------|-------|-------|
| Reduce redundant scans / stuck loops | Both | Deterministic fallback |
| Step-count or runtime metrics in tests | Both | Guard against b05s regression |
| HLD PDF for submission | Both | From `docs/HLD.md` |
| `bonus.txt` | Both | Only if claiming bonus |
| Final multi-scenario sanity run | Both | Clean logs, reproducible scores |

**Gate D:** Optional polish complete; submission package ready.

---

## Definition of done

- [ ] Exact component names; frozen interfaces unchanged
- [ ] `drone_mapper_simulation` CLI behavior per assignment
- [ ] `drone_mapper_simulation_test` with all required `--gtest_filter` prefixes
- [ ] `simulation_output.yaml` + `output_results/` documented in `readme.txt`
- [ ] Immediate error logging; score -1 on failed continuable scenarios
- [ ] No ex1 anti-patterns (`docs/ex1-mistakes.md`)
- [ ] Small maps finish in reasonable time
- [ ] `docs/HLD.md` (and PDF) match implementation

## Risks

| Risk | Mitigation |
|------|------------|
| Late test harness | Create `tests/` + CMake in Phase 1, not Phase 4 |
| `.npy` bugs | Golden tests on `data_maps/` first |
| Algorithm timeout (b05s) | Profile in Phase 2; cap work per `nextStep` |
| Copying bad ex1 patterns | `ex1-anti-patterns` rule + code review each PR |
| Skeleton API drift | Never edit "Do not change" headers |

## Quick reference

```bash
cmake --preset default && cmake --build --preset default
./build/drone_mapper_simulation [simulation.yaml] [output_path]
./build/drone_mapper_simulation_test --gtest_filter=DroneControl.*
./build/maps_comparison origin.npy target.npy
```

Component porting details: `refactor_by_component.md`  
Agent rules: `.cursor/rules/` · Skills: `.cursor/skills/`
