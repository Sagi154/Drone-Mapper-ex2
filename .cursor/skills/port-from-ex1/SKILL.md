---
name: port-from-ex1
description: Ports algorithm logic from ex1 Drone-Mapper into ex2 skeleton components. Use when reusing exploration, collision, or scoring ideas from ../Drone-Mapper/ — not when copying ex1 classes or architecture.
---

# Port from Ex1

**Ex1 is logic reference only.** On conflict: Assignment 2 + skeleton win. See `docs/ex1-mistakes.md`.

## Workflow

1. Identify ex1 **logic** to reuse (frontier heuristic, collision math, scoring formula).
2. Map to ex2 component via `docs/ex1-to-ex2-porting.md`.
3. Check skeleton — does it already provide this? (`MockLidar`, `ScanResultToVoxels`)
4. **Rewrite** into ex2 types and control flow — never copy ex1 class names.

## Control flow change

| Ex1 | Ex2 |
|-----|-----|
| `DroneAlgorithm::tick()` loop in main | `MappingAlgorithmImpl::nextStep(state, scan)` |
| Monolithic `SimulationState` | `DroneState` + `IMap3D` / `IMutableMap3D` |
| Text configs | YAML parsers |
| Text maps | `.npy` via `Map3DImpl` |

## Do not port

`ScanProbing`, `ScanOptions`, `TickSnapshot`, `SimulationState`, ex1 `ErrorLogger` (especially `flushIfNeeded` at end of run), ex1 `main`, per-function `namespace su = ...`.

## Error handling when porting

| Ex1 pattern | Ex2 replacement |
|-------------|-----------------|
| `logger.add(...)` then `flushIfNeeded` at shutdown | Append to per-run log **on every** error |
| `return 1` from main on bad start pose / map | `ErrorRef` + `score: -1` for that run; continue composition |
| Free-form `"unrecoverable error: ..."` on stderr only | Structured `ErrorRef` in log **and** YAML report |

Ex1 recoverable/unrecoverable **semantics** carry forward; ex2 adds immediate logging and multi-run continuation.

## Safe to port conceptually

- Frontier/BFS → `MappingAlgorithmImpl::nextStep`
- Collision / passability → `MockMovement`
- Scoring math → `MapsComparison`

## After each port chunk

1. Run component test filter for affected component.
2. Scan new code against `docs/review-error-codes.md` (e03, e07, e08, e16, e17, e22).
3. Profile small-map runtime if algorithm changed (b05).

## Full mapping table

See `refactor_by_component.md`.
