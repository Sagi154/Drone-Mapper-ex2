# Assignment 2 — Work Plan

**Deadline:** July 1, 2026, 23:30  
**Team:** 2 people · **Repo state:** Phase 3 complete; Gate B verified on `main` (orchestration PRs #19–#22 merged, CI green)

## Goal

Ship a complete Assignment 2 submission: mandatory components, YAML/.npy I/O, tests with required GTest filters, graceful error handling, and a working mapping algorithm. **Algorithm strategy:** port the ex1 algorithm logic as-is into the ex2 skeleton interfaces — no redesign or efficiency work needed. The algorithm was not the problem; failures during testing were caused by the maps used, not by the algorithm itself.

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
| **Both** | Integration, Gate B/C review, Phase 5 submission polish, HLD/docs |

**How to read "Owner":** primary implementer and reviewer for that component — **not** the only person working that week. Phases 2 and 3 overlap on purpose so neither person sits idle waiting on the other.

Use Cursor skills: `implement-ex2-component`, `port-from-ex1`, `write-component-test`, `pre-submission-review`.

---

## Timeline (overlapping phases)

```
Day:  0    1    2    3    4    5    6    7    8    9   10   11   12
      |---- Phase 0 ----|
           |-------- Phase 1 --------|
                        |----------- Phase 2 (runtime) -----------|
                             |----------- Phase 3 (orchestration) -----------|
                                                  |---- Phase 4 ----|
                                                                   |-- Phase 5 --|
                                        ^ handoff: MissionControl::runMission
```

| Phase | Window | Primary focus | Both work on |
|-------|--------|---------------|--------------|
| 0 | Day 0–1 | Bootstrap | Build, read docs |
| 1 | Days 1–3 | Foundation | A: I/O · B: mocks + test harness |
| 2 | Days 3–6 | Runtime loop | B leads · A starts factory stubs + scoring fixtures |
| 3 | Days 4–8 | Orchestration | A leads · B finishes runtime tests + factory wiring help |
| 4 | Days 7–9 | Tests & compliance | Integration, HLD, readme |
| 5 | Days 9–12 | Efficiency & submission | After Gate C only |

---

## Handoffs (do not skip)

| # | Trigger | From → To | Deliverable | Status |
|---|---------|-----------|-------------|--------|
| H1 | Phase 1 Gate A passes | A → B | `Map3DImpl` loads `data_maps/*.npy`; mission/drone YAML parses to typed configs | ✅ Done |
| H2 | `DroneControlImpl::step` compiles | B → A | B shares minimal call signature + deps list for factory wiring | ✅ Done |
| H3 | `MissionControlImpl::runMission` runs one mission | B → A | A plugs runtime into `SimulationRunImpl` / minimal `SimulationRunFactoryImpl` | ✅ Done |
| H4 | Gate A (full) passes | Both | One scenario completes end-to-end and writes output `.npy` | ✅ Done |
| H5 | Gate B passes | A → B | Full CLI path works; B adds `Integration.*` tests against real wiring | ✅ Done |

**Pair session (~30 min) at H2:** walk through the DI graph in `docs/HLD.md` factory sequence so A's stub factory matches B's constructor needs.

---

## Phase 0 — Bootstrap (Day 0–1) ✅ DONE

**Outcome:** Clean build from repo root; dev workflow agreed.

| Task | Owner | Deliverable |
|------|-------|-------------|
| Verify vcpkg + CMake preset build | Both | `cmake --preset default && cmake --build --preset default` green |
| Read `docs/assignment2-checklist.md`, `refactor_by_component.md` | Both | Shared understanding of gates |
| Run skeleton binaries once | Both | Note stub behavior baseline |
| Agree `output_results/` naming in `readme.txt` draft | A | Section in readme |

**Gate 0:** `drone_mapper_simulation` and `maps_comparison` executables build.

---

## Phase 1 — Foundation (Days 1–3) ✅ DONE

**Outcome:** Maps load/save; configs parse; errors log immediately.

| Task | Owner | Files / notes |
|------|-------|---------------|
| `Map3DImpl` — TinyNPY load/save, `MapConfig`, `isInBounds` | A | `src/Map3DImpl.cpp` · golden tests on `data_maps/*.npy` |
| YAML parsers — drone, mission, lidar, simulation, composition | A | New `src/io/` (or similar) · link `yaml-cpp` in `CMakeLists.txt` |
| Error logger — structured codes, no exposed internals (e22) | A | New `src/io/` · immediate flush |
| `MockGPS` + `MockMovement` | B | Port collision ideas from ex1 · no ex1 class names |
| Component tests: `Map3DImpl` helpers, GPS, movement | B | Start `tests/components/` + CMake `drone_mapper_simulation_test` |
| Draft `SimulationRunFactoryImpl` interface / empty stub | A | Compile-only skeleton · unblocks Phase 2 overlap |

**Gate A:** Load a `data_maps/*.npy` map, parse a hand-written YAML, log a test error to file.

**Ex1 watchlist:** no `ErrorLogger::lines()`, no per-function mp-units aliases (e17s), strong types at parse boundary (e16).

---

## Phase 2 — Runtime loop (Days 3–6) ✅ DONE

**Outcome:** Single mission runs: step loop, scan, voxels, save map.  
**Overlaps Phase 3 from Day 4** — A does not wait for all of Phase 2 to finish before starting orchestration stubs.

### Person B (primary — runtime) ✅ DONE

| Task | Files / notes |
|------|---------------|
| `MappingAlgorithmImpl::nextStep` — port ex1 algorithm as-is | Port **logic** from `ExplorationFrontier` · adapt to ex2 interfaces · no redesign needed (algorithm was correct; past failures were map-related) |
| `DroneControlImpl::step` — movement → scan → voxels | Wire `ScanResultToVoxels` · first step `latest_scan == nullptr` |
| `MissionControlImpl::runMission` — max_steps, status, save | `MissionRunResult` + errors |
| `MapsComparison` + `maps_comparison` exe | stdout score only · stderr on error |
| Component tests | `DroneControl.*`, `MissionControl.*`, `MappingAlgorithm.*`, `MapsComparison.*` |

### Person A (parallel — unblocks Phase 3) ✅ DONE

| Task | Files / notes |
|------|---------------|
| Minimal `SimulationRunFactoryImpl` — single-scenario wiring | Hard-code one tuple first · expand in Phase 3 → **expanded to full DI on `main` (Phase 3 overlap)** |
| `SimulationRunImpl` skeleton — call `runMission` when H3 lands | Score -1 placeholder until compare wired → **full `runMission` + `MapsComparison` on `main` (Phase 3 overlap)** |
| Test fixtures using parsed YAML + loaded `.npy` | Reuse A's parsers · `tests/support/ConfigFixtures.hpp` + E2E tests (unmerged) |
| `output_results/` path layout draft | Document in `readme.txt` early → **also covers Phase 3 `runOutputDir` / `error.log` spec** |

**Gate A (full):** ✅ PASS — factory + runtime complete one mission on a tiny map, write `output_map.npy`, and score via `MapsComparison` (factory + manager E2E tests).

**Performance note:** The ex1 algorithm itself is not slow. Past b05 concerns were caused by the specific maps used in testing. That said, **tests must still complete within ~10 s** — use small/synthetic maps in unit and component tests, and verify no single test blocks the suite.

---

## Phase 3 — Orchestration (Days 4–8) ✅ DONE

**Outcome:** Full composition cartesian product; CLI; report files.  
**Starts while Phase 2 runtime is still landing** — factory starts as single-scenario stub, then generalizes.

**Gate B verified on `main`** — happy path, bad-map `-1`, continue-after-failure, and mid-mission `error.log` mirroring all pass in CI and manual smoke.

### Already done during Phase 2 overlap

Work landed while Phase 2 runtime was still merging — mapped back to Phase 3 tasks:

| Phase 3 task | What was done in Phase 2 | Where |
|--------------|--------------------------|-------|
| `SimulationRunFactoryImpl` — full DI graph | Started as minimal stub; grew to full wiring (hidden map load, mocks, algorithm, drone/mission control) | `main` |
| `SimulationRunImpl` — mission + compare + `SimulationResult` | `run()` calls `runMission`, scores via `MapsComparison`, `-1` on errors | `main` |
| `SimulationManager` — cartesian product + report | Loop + `simulation_output.yaml` writer | `main` |
| `drone_mapper_simulation_main` — CLI | `parseSimulationCliArgs`, `parseCompositionFile`, stderr on startup failure | `main` |
| `output_results/` layout + path helpers | `runOutputDir` / `runOutputMap` / `runErrorLog` in `readme.txt` + `src/io/` | `main` |
| Test fixtures (parsed YAML + `.npy`) | `ConfigFixtures.hpp`, `composition_e2e.yaml`, factory + manager E2E tests | `main` |
| Factory output-map allocation | `makeEmptyOutputArray` — output map must be allocatable before save | `main` |
| Person B — H3 factory wiring | `MissionControlImpl::runMission` + factory output-map config fix | `main` |
| Person B — orchestration component tests | `SimulationManager.*` (GMock), `SimulationRun.*` (GPS/movement + mock `run`) | `main` |
| Person B — real-wiring E2E test | `SimulationRun.Factory_EndToEnd_*`, `SimulationManagerTest.RealFactory_*` | `main` |

### Person A (primary — orchestration) ✅ DONE

| Task | Files / notes | Status |
|------|---------------|--------|
| Merge factory output-map fix + E2E tests | `SimulationRunFactoryImpl.cpp`, `test_simulation_run_factory.cpp`, `test_simulation_manager.cpp` | Done |
| Missing-input handling (b06) | Missing `simulation.yaml`, bad map path, corrupt `.npy`, invalid composition refs | Done — PR #22 |
| Runtime mission errors → `error.log` | Mirror `mission_results[].errors` in per-run log via `SimulationRunImpl` | Done — PR #21 |
| Gate B verification | Manual CLI smoke: composition YAML → `simulation_output.yaml` + `output_results/` | Done |

### Person B (parallel — finish runtime + support wiring) ✅ DONE

| Task | Files / notes | Status |
|------|---------------|--------|
| H3 factory wiring | Runtime plugged into `SimulationRunFactoryImpl` / `SimulationRunImpl` | Done |
| Orchestration component tests | `SimulationManager.*`, `SimulationRun.*` (mocks + real wiring) | Done |
| Real-wiring E2E tests | `SimulationRun.Factory_EndToEnd_*`, `SimulationManagerTest.RealFactory_*` | Done |
| Gate B support / verification | CLI path verified on `main`; scenario `-1` / continue-after-failure / `error.log` | Done |
| `Integration.*` tests | Real algorithm + mock algorithm (`tests/integration/`) | Not started — Phase 4 (H5 handoff) |

**Gate B:** Run `./drone_mapper_simulation` with a small composition YAML; get `simulation_output.yaml` + `output_results/`; failed scenario gets -1 and run continues. **Verified** — see `tests/data/configs/composition_continue_after_failure.yaml` and `SimulationManagerTest.RealFactory_ContinuesAfterStartupFailure`.

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

## Phase 5 — Submission polish (Days 9–12, buffer)

**Start only after Gate C.**

> **Algorithm note:** No efficiency work planned. The ex1 algorithm is kept as-is, refactored to fit ex2 interfaces. The previous timeout concern was caused by bad test maps, not the algorithm itself.

| Task | Owner | Notes |
|------|-------|-------|
| Verify algorithm correctness on assignment-provided maps | Both | Confirm ex1 port behaves correctly end-to-end |
| HLD PDF for submission | Both | From `docs/HLD.md` |
| `bonus.txt` | Both | Only if claiming bonus |
| Final multi-scenario sanity run | Both | Clean logs, reproducible scores |

**Gate D:** Submission package ready.

---

## Definition of done

- [x] Exact component names; frozen interfaces unchanged
- [x] `drone_mapper_simulation` CLI behavior per assignment
- [ ] `drone_mapper_simulation_test` with all required `--gtest_filter` prefixes
- [x] `simulation_output.yaml` + `output_results/` documented in `readme.txt`
- [x] Immediate error logging; score -1 on failed continuable scenarios
- [ ] No ex1 anti-patterns (`docs/ex1-mistakes.md`)
- [x] Small maps finish in reasonable time
- [ ] `docs/HLD.md` (and PDF) match implementation

## Risks

| Risk | Mitigation |
|------|------------|
| Late test harness | Create `tests/` + CMake in Phase 1, not Phase 4 |
| `.npy` bugs | Golden tests on `data_maps/` first |
| Test timeout (b05s) | Algorithm is fine; risk is map size in tests — use small/synthetic maps; enforce ~10 s per test |
| A blocked waiting on B's runtime | A starts factory stub + fixtures in Phase 2 overlap (see timeline) |
| Factory/runtime interface mismatch | H2 pair session on DI graph before A generalizes factory |
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
