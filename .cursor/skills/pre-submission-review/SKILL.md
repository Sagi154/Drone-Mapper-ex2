---
name: pre-submission-review
description: Runs the Assignment 2 and AdvCpp review checklist before commit or submission. Use when finishing a feature, preparing a PR, or validating submission readiness for Drone-Mapper-ex2.
---

# Pre-Submission Review

Copy this checklist and verify each item against the live code, output, and docs.

---

## Build (b01, b02, e11)

- [ ] `cmake --preset default && cmake --build --preset default` succeeds with zero errors
- [ ] All three targets build: `drone_mapper_simulation`, `drone_mapper_simulation_test`, `maps_comparison`
- [ ] All dependencies declared in `vcpkg.json` — no manual install step (e11)

---

## Program run (b03–b08)

- [ ] `./build/drone_mapper_simulation` completes on default scenarios from CWD (b03)
- [ ] Same from alternate `output_path` and alternate composition YAML path (b07)
- [ ] Minor valid config edits (e.g. change `max_steps`, tweak resolution) still pass (b07)
- [ ] Invalid/corrupt config exits gracefully — no crash, no `exit()` call (b04)
- [ ] CLI path resolution works correctly (b08):
  - Missing yaml arg → uses `simulation.yaml` in CWD
  - Filename only → resolves under CWD
  - Relative path → resolves under CWD
  - Absolute path → used as given
  - Missing `output_path` → defaults to CWD
- [ ] Missing `simulation.yaml` / missing map files → error logged immediately, graceful return from `main` (b06)
- [ ] Component/unit tests: synthetic/small maps finish in ~10 s each (b05)
- [ ] Integration tests (instructor focused compositions + optional `composition_benchmark.yaml`): finish within **1 minute** (b05)

---

## Tests

- [ ] `./build/drone_mapper_simulation_test` — all tests pass
- [ ] All 8 required filter prefixes work from the umbrella binary:
  `SimulationManager.*`, `SimulationRun.*`, `MissionControl.*`, `DroneControl.*`,
  `MappingAlgorithm.*`, `MockLidar.*`, `MapsComparison.*`, `Integration.*`
- [ ] Component tests live in `tests/components/`, integration tests in `tests/integration/`
- [ ] At least 2 integration tests: one with real `MappingAlgorithmImpl`, one with mock algorithm
- [ ] Bug-catch readiness (assignment §Tests):
  - Each component suite catches ≥60% of injected bugs for that component (Review Guideline cites >50% full / >25% partial)
  - Integration tests catch ≥20% of injected bugs
  - Breaking one component does **not** fail unrelated component suites (isolation)
  - Integration tests exercise the end-to-end path (instructor maps in `tests/data/instructor/`)

---

## Integration benchmark

- [ ] Run `./build/drone_mapper_simulation tests/data/configs/composition_benchmark.yaml <tmp_out>` locally
- [ ] Completes in ≤1 min; `simulation_output.yaml` written; `output_results/run_0001/` created
- [ ] Hidden `uint8` maps (`scenario_house.npy`, `benchmark_map.npy`): stored values `2`, `3`, `4`, `18`, `45`, … read as `Occupied` — not `Unmapped` (see `docs/map3d_impl_contract.md`; regression: `Map3DImpl.StoredValuesGreaterThanOneReadAsOccupied`)
- [ ] `int8` output maps: `Unmapped` (`-1`) cells remain `Unmapped` on read (regression: `Map3DImpl.Int8UnmappedCellsRemainUnmapped`)
- [ ] Instructor focused compositions (`tests/data/instructor/compositions/composition_{small_room,big_room,house_lower}.yaml`) complete without timeout; `mission_score >= 90`

---

## Outputs

- [ ] `simulation_output.yaml` structure matches `readme.txt`:
  - `score_report`, `runs[]`, `config_indices`, `mission_results`, `errors[]`
  - `resolution_request_status` present per run (`ACCEPTED | IGNORED | IGNORED TOO SMALL`)
  - `run_id` is 1-based; `run_dir` is `output_results/run_NNNN`
- [ ] `output_results/run_NNNN/output_map.npy` and `error.log` present per run
- [ ] Run folder naming: `run_0001`, `run_0002`, …; loop order is **(simulation, mission) pairs × drones × lidars**
- [ ] Failed scenarios: `mission_score: -1`, error in `error.log`, later runs still execute
- [ ] Failed run with zero steps: `output_map.npy` omitted; failed run with ≥1 steps: partial map saved
- [ ] Error logs written **at occurrence time** — timestamped lines, not batched at end
- [ ] Runtime `ErrorRef::code` strings used (e.g. `MAP_FILE_NOT_FOUND`) — **never** rubric codes (`e05`, `b04`, …)
- [ ] `error.log` format: `<ISO-8601 UTC> <ERROR_CODE> <user-facing message>` (one line per error)
- [ ] Errors in `error.log` mirror `mission_results[].errors` in `simulation_output.yaml`

---

## Code review (spot-check)

- [ ] No per-function `namespace su = mp_units::...` aliases (e17s — severe deduction; import once at file scope or use `Units.h`)
- [ ] No raw `double` cm/deg in APIs; strong types (`PhysicalLength`, angles) at boundaries (e16)
- [ ] No mp-units unwrap for math (`.numerical_value_in(...)`) (e03)
- [ ] `const` methods and `const`-ref params wherever possible (e06, e07)
- [ ] No exposed internal storage (no public getters returning backing containers) (e22)
- [ ] No unneeded public methods or data members beyond skeleton interfaces (e08)
- [ ] No magic numbers — named constants or `mp_units` literals only (e23)
- [ ] Functions are reasonably short / single responsibility (e09)
- [ ] No duplicate logic or near-identical flows (e10)
- [ ] No raw `new`/`delete`/`malloc`; smart pointers and RAII throughout (e13)
- [ ] No repeated computation of the same value (e21)
- [ ] No unnecessary `#include` or library dependencies (e17)
- [ ] Header grouping: no unrelated classes in one header; no overly granular splits (e01, e02)

---

## Ex1 anti-patterns (walk `docs/ex1-mistakes.md`)

- [ ] No `namespace su = mp_units::si::unit_symbols` in individual functions (e17s)
- [ ] No `ScanProbing`, `ScanOptions`, `TickSnapshot` types ported from ex1
- [ ] No `SimulationState` monolithic bag
- [ ] No `ErrorLogger::lines()` or any exposed log-line getter (e22)
- [ ] No deferred error flush (`flushIfNeeded` pattern) — every error written immediately
- [ ] No whole-program abort when a single scenario fails (`return 1` from main mid-composition)
- [ ] `setTruthCell` or similar test-only setters not in production API (e08)
- [ ] `MockLidar::scan` and similar read-only methods are `const` (e07)

---

## Docs

- [ ] `docs/HLD.md` class diagram and sequence diagrams match current code (e14, e15)
- [ ] HLD covers all components: `SimulationManager`, factory, `SimulationRunImpl`, `MissionControlImpl`, `DroneControlImpl`, `MappingAlgorithmImpl`, `MockLidar`, `MapsComparison`, CLI, error-logging flow
- [ ] `readme.txt` matches actual build commands, output layout, and error-handling behavior
- [ ] `bonus.txt` present **only** if claiming bonus features

---

## maps_comparison

- [ ] Identical maps → stdout `100` (or `100.0`)
- [ ] Very similar maps → near 100 but **not** 100; very different → near 0 (not necessarily 0)
- [ ] Success: stdout = score only, no extra text
- [ ] Error: stdout `-1`, descriptive message on **stderr**
- [ ] `comparison_config=<path>` optional third argument accepted and parsed

---

## Submission package (Phase 5 — after Gate C)

- [ ] All 8 `--gtest_filter` prefixes pass from umbrella `drone_mapper_simulation_test`
- [ ] `sim_benchmark.yaml` integration timing: ≤1 min clean; also ≤1 min with per-component bug injections where feasible
- [ ] HLD PDF exported from `docs/HLD.md` and included for submission
- [ ] A-side ex1 anti-patterns walk (see section above) — `docs/ex1-mistakes.md` confirmed clean
- [ ] Gate C joint sign-off: both team members ran full test suite; all ≥1-min scenarios timed

---

## Reference

- `docs/assignment2-checklist.md` — mandatory requirements
- `docs/review-error-codes.md` — rubric codes + severities
- `docs/ex1-mistakes.md` — ex1 deductions to avoid
- `docs/map3d_impl_contract.md` — `.npy` dtype rules, uint8 clamp, int8 enum, voxel mapping
- `readme.txt` — agreed output layout + `simulation_output.yaml` schema
