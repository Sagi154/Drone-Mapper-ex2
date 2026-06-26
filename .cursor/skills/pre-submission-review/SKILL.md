---
name: pre-submission-review
description: Runs the Assignment 2 and AdvCpp review checklist before commit or submission. Use when finishing a feature, preparing a PR, or validating submission readiness for Drone-Mapper-ex2.
---

# Pre-Submission Review

Copy this checklist and verify each item.

## Build (b01, b02, e11)

- [ ] `cmake --preset default && cmake --build --preset default` succeeds
- [ ] Targets build: `drone_mapper_simulation`, `drone_mapper_simulation_test`, `maps_comparison`
- [ ] Dependencies only via `vcpkg.json` — no manual install steps

## Program run (Review Guideline, b03–b08)

- [ ] `./build/drone_mapper_simulation` completes on default scenarios (CWD)
- [ ] Same from alternate `output_path` / composition path
- [ ] Minor valid config edits still pass
- [ ] Invalid config exits gracefully (no crash)
- [ ] Component/unit tests: small synthetic maps finish in ~10 s each (b05)
- [ ] Integration tests (including instructor-provided scenarios): finish within 1 minute
- [ ] Missing `simulation.yaml` / map files handled with error log (b06)

## Tests

- [ ] `./build/drone_mapper_simulation_test` — all pass
- [ ] Each filter works: `SimulationManager.*`, `SimulationRun.*`, `MissionControl.*`, `DroneControl.*`, `MappingAlgorithm.*`, `MockLidar.*`, `MapsComparison.*`, `Integration.*`

## Outputs

- [ ] `simulation_output.yaml` structure matches `readme.txt` (`score_report`, `runs[]`, `config_indices`)
- [ ] `output_results/run_NNNN/output_map.npy` and `error.log` per run (paths match YAML)
- [ ] `run_id` ↔ folder naming consistent (`run_0001` = `run_id: 1`)
- [ ] Failed scenarios: `mission_score: -1`, error in `error.log`, later runs still execute
- [ ] Error logs written **at occurrence time** (timestamped lines, not batch at end)
- [ ] Runtime `ErrorRef::code` values — not rubric codes (`e05`, `b04`, …)

## Code review codes (spot-check)

- [ ] No per-function `namespace su = mp_units::...` (e17s)
- [ ] No raw `double` cm/deg in APIs (e16)
- [ ] No mp-units unwrap for math (e03)
- [ ] const methods and const-ref params (e06, e07)
- [ ] No exposed internal containers (e22)
- [ ] No unneeded public methods (e08)
- [ ] No magic numbers (e23)

## Docs

- [ ] `docs/HLD.md` class + sequence diagrams match code (e14, e15)
- [ ] `readme.txt` matches build/run commands
- [ ] `bonus.txt` present only if claiming bonus

## maps_comparison

- [ ] Identical maps → 100
- [ ] stdout score only; errors → stdout -1, stderr message

## Reference

- `docs/assignment2-checklist.md`
- `docs/review-error-codes.md`
- `docs/ex1-mistakes.md`
