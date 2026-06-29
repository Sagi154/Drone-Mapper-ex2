# Ex1 → Ex2 Porting Guide

**Ex1 is reference only.** On conflict: Assignment 2 skeleton APIs + Review Guideline win. See `docs/ex1-mistakes.md` for patterns to avoid.

Full table: `refactor_by_component.md`.

## Do not reimplement

- `MockLidar` — use skeleton as-is
- `ScanResultToVoxels` — integrate as provided

## Component mapping (summary)

| Ex2 | Ex1 source | Reuse | Notes |
|-----|------------|------:|-------|
| `MockGPS` | `PositionMock` | ~95% | Adapt to `gps_resolution` constructor |
| `MockMovement` | `MovementMock`, `CollisionDetector` | ~85% | Wire via `IDroneMovement` + `MockGPS` |
| `MappingAlgorithmImpl` | `DroneAlgorithm`, `ExplorationFrontier` | ~70% | `tick()` → `nextStep(state, scan)`; **no** ScanProbing/TickSnapshot |
| `DroneControlImpl` | ex1 runtime loop | ~50% | Split: execute command, scan, voxels |
| `MissionControlImpl` | ex1 `main` loop | ~40% | `runMission()`, max steps, save map |
| `Map3DImpl` | `BuildingMap` | ~40% | `.npy` + `MapConfig`; not `SimulationState` |
| `MapsComparison` | `Scorer` | ~70% | Ex2 map interfaces + optional comparison YAML |
| `SimulationRunImpl` | ex1 result assembly | ~30% | Mission + compare + `SimulationResult` |
| `SimulationRunFactoryImpl` | ex1 DI in main | ~30% | Per-scenario object graph |
| `SimulationManager` | (new) | ~20% | Aligned pair expansion + report |
| YAML layer | ex1 txt parsers | ~50% | Same validation spirit, YAML format |
| `.npy` I/O | ex1 text maps | ~20% | New implementation |

## Recommended order

1. `Map3DImpl`, YAML parsing, `.npy` I/O
2. `MockGPS`, `MockMovement`, `MappingAlgorithmImpl`, `DroneControlImpl`, `MissionControlImpl`
3. `SimulationRunFactoryImpl`, `SimulationRunImpl`, `SimulationManager`, main
4. `MapsComparison`
5. Component tests, then integration tests
6. Algorithm efficiency (after mandatory gate)
