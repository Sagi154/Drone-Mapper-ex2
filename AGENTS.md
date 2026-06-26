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
| `context/` | Original assignment and review docx files |

## Build and run

```bash
cmake --preset default
cmake --build --preset default
./build/drone_mapper_simulation [<simulation.yaml>] [<output_path>]
./build/drone_mapper_simulation_test
./build/maps_comparison <origin_map> <target_map> [comparison_config=<path>]
```

## Grading (Review Guideline)

Submissions are graded by **bug injection** against your test suite (`context/Exercise 2 Review Guideline.docx`):

- **>50%** of injected bugs caught per component = full marks; **>25%** = partial marks.
- Isolate tests per component where possible; some bugs may legitimately fail multiple suites.
- Integration tests use instructor-provided config/map files; must finish within **1 minute** (with and without bugs).
- Manual code review is lighter than ex1 — comments in most cases; only extremely poor practices affect the grade.

## Hard rules

- **Never** change skeleton interfaces marked "Do not change".
- **Never** reimplement `MockLidar` or `ScanResultToVoxels`.
- **Never** use `exit()` / `abort()`; return from `main` gracefully.
- **Never** unwrap mp-units to `double` for math; use quantity arithmetic.
- Ex1 (`../Drone-Mapper/`) is **logic reference only** — on conflict, Assignment 2 + Review Guideline win.
- Log all errors **immediately** to the per-run error log; never defer (do not port ex1 `flushIfNeeded`).
- **Rubric codes** (`e05`, `b04`, …) ≠ **runtime codes** (`ErrorRef::code` in YAML) — see `docs/review-error-codes.md`.
- One failed scenario → `score: -1` and continue; do not abort the whole composition unless startup is impossible.
- Keep `docs/HLD.md` aligned with code (graded).

## Test filters (required names)

`SimulationManager.*`, `SimulationRun.*`, `MissionControl.*`, `DroneControl.*`, `MappingAlgorithm.*`, `MockLidar.*`, `MapsComparison.*`, `Integration.*`
