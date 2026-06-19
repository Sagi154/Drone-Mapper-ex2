# Ex1 Mistakes — Do Not Repeat in Ex2

Ex1 final grade: 76.0 (bonus 2.0). Ex1 is **logic reference only**; these patterns caused real deductions.

## Manual code review

| Code | Issue | Ex1 example | Ex2 fix |
|------|-------|-------------|---------|
| e17s | `namespace su = mp_units::si::unit_symbols` in many functions | Scattered aliases | Import via `Units.h` once per file; no local mp-units aliases |
| e16n | Raw numerics instead of strong types | `ScanProbing`, `ScanOptions`, `TickSnapshot` | Use `types::DroneState`, `types::MappingStepCommand`, `PhysicalLength`, angles from `Units.h` |
| e22m | Exposing internal storage | `ErrorLogger::lines()` | Log API with `add`/`log`; no public getter to backing container |
| e08m | Unneeded public API | `SimulationState::setTruthCell` | Only methods required by skeleton interfaces; test helpers in fixtures |
| e07m | Missing const | `LidarMock::scan` | Mark non-mutating methods `const` |

## Program run

| Code | Issue | Ex2 fix |
|------|-------|---------|
| b05s | Scenario timeout (~1 min) | Profile small maps (~10s target); avoid redundant rescans; cap work in `nextStep` |
| b06m | Missing input not handled | Test missing `simulation.yaml`, map files; graceful error + immediate log + score -1 |

## Do not port from ex1

- `ScanProbing`, `ScanOptions`, `TickSnapshot`
- `SimulationState` as monolithic state bag
- `ErrorLogger` with exposed `lines()`
- Ex1 `main.cpp` orchestration loop
- Text config parsers (rewrite for YAML)
- Per-function `namespace su = ...` aliases

## Safe to port conceptually

- Frontier/BFS exploration → `MappingAlgorithmImpl::nextStep`
- Collision / passability → `MockMovement`
- Scoring math ideas → `MapsComparison` (adapt to `.npy` + `MapConfig`)
