# Assignment 2 — Mandatory Checklist

Condensed from `context/Advanced Topics TAU 2026B - Assignment 2.docx`. On conflict, the docx wins.

## Components (exact names, all mandatory)

- `drone_mapper_simulation_main` — minimal; creates `SimulationManager`
- `SimulationManager`, `SimulationRunImpl`, `SimulationRunFactoryImpl`
- `MissionControlImpl`, `DroneControlImpl`, `MappingAlgorithmImpl`
- `MockGPS`, `MockMovement`, `Map3DImpl`, `MapsComparison`
- Use provided `MockLidar` and `ScanResultToVoxels` — integrate, do not redesign

## Inputs

- Maps: `.npy` binary (NumPy voxel format)
- Configs: YAML — `drone_config`, `mission_config`, `lidar_config`, `simulation_config`, `simulation_compositions`, optional `comparison_config`
- Composition: cartesian product of simulations × mission_configs × drone_configs × lidar_configs

## Outputs

- `simulation_output.yaml` — hierarchical score report (see assignment example)
- `output_results/` — maps and error logs (document layout in `readme.txt`)
- Per-run error log — all errors logged **immediately** when they occur

## CLI

```text
./drone_mapper_simulation [<simulation.yaml>] [<output_path>]
```

- Missing yaml arg → use `simulation.yaml` in CWD
- Filename only → look in CWD
- Relative path → under CWD
- Absolute path → use as given
- Missing `output_path` → CWD

## Maps comparison utility

```text
./maps_comparison <origin_map> <target_map> [comparison_config=<path>]
```

- stdout: score only (0–100 float)
- error: stdout `-1`, descriptive message on stderr
- Identical → 100; very similar → near 100 but not 100; very different → near 0

## Tests

- Component tests: `tests/components/` — SimulationManager, SimulationRun (+ MockGPS/MockMovement), MissionControl, DroneControl, MappingAlgorithm, MockLidar, MapsComparison
- Integration: `tests/integration/` — at least 2 (real algorithm + mock algorithm)
- Run: `./drone_mapper_simulation_test` with required `--gtest_filter` prefixes

## Error handling (ex1 + ex2)

- Never crash; never call `exit()`
- Recoverable input errors → defaults + **immediate** log (no deferred flush like ex1 `input_errors.txt`)
- Unrecoverable **for one run** → structured `ErrorRef`, `score: -1`, continue composition
- Unrecoverable **for whole program** (e.g. composition YAML unreadable) → log, return from `main` gracefully
- Group failure (e.g. bad map) → `-1` for all runs in group
- Rubric codes (`e05`, `b04`) ≠ runtime `error_ref.code` strings — see `docs/review-error-codes.md`

## API constraints

- Use exact skeleton interfaces and types — no deviation
- `IMappingAlgorithm::nextStep(state, latest_scan)` — `latest_scan` may be `nullptr` on first step
- `MapConfig` (boundaries, offset, resolution) on `IMap3D`; `MissionConfigData` has no boundaries
- `DroneControlImpl::step()`: movement first, then scan, voxels via `ScanResultToVoxels`

## Performance

- Working algorithm with reasonable efficiency (not required to be optimal)
- Small maps should finish in ~10s; >2 min on small maps may lose points
- Grading timeout: ~1 minute per scenario (b05)

## Submission docs

- `readme.txt` — build/run, output formats
- `docs/HLD.md` (+ PDF for submission) — class + sequence diagrams matching code
- `bonus.txt` — only if claiming bonus features

## Assumptions (carry from ex1 unless overridden)

- Perfect sphere drone; no battery/recharging
- Deterministic algorithm (no randomness)
- Single supported output resolution per run (simulation may IGNORE unsupported requests)
