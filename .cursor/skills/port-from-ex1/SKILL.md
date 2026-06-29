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

---

## ⚠️ MappingAlgorithmImpl — behavioral contract checklist

Porting ex1's frontier/BFS into `nextStep` is not enough. Ex1 worked because scanning, fusion, and footprint together guaranteed a passable local bubble before planning. That contract must be preserved in ex2 or the drone gets stuck at the start.

**Verify all of the following before calling the port done:**

### 1. Local neighborhood must be Empty before first plan

`isSpherePassable` (frontier BFS) requires **every voxel within drone radius** to be non-`Unmapped` for the drone to move. If even one neighbor cell is `Unmapped`, movement is blocked.

- In ex1, `SphericalScanner` fused a full hemisphere **in one tick**, clearing a passable bubble before planning ever ran.
- In ex2, orientations drip one-per-step. After a coarse pass, if adjacent cells are still `Unmapped`, the drone cannot move.
- **Check:** after a full scan pass, does `isSpherePassable` return true at start position? If not, movement is dead.

### 2. Footprint stamping vs grid resolution

`markDroneFootprintEmpty` only stamps cells whose **centers** fall within drone radius.

- If `drone.radius < grid_resolution`, only the center voxel is stamped.
- With `dimensions_cm: 10` → `radius = 5 cm` and grid `10 cm`, **zero neighbors** are stamped by footprint.
- Footprint alone is not enough to unblock movement in this config.

### 3. Coarse scan pass must produce enough coverage

The coarse pass (`kCoarsePassAngularScale = 2.0`) produces `~ceil(180 / (2 * el_step))` elevation tiers. For a 5 cm radius / 10 cm grid:

- `el_step_base ≈ atan(10/5) ≈ 63°`
- Coarse → `el_step ≈ 127°` → **~3 elevations, ~6 orientations total**

Six scan steps mapping only the nearest wall cells is not enough to open adjacent voxels. Verify the orientation count is sufficient for the map geometry in use.

### 4. `diagnose()` bails immediately when start is not passable

`MappingAlgorithmFrontier::diagnose` returns early if `isSpherePassable(start) == false`, setting `explore_path_available = false`. This means the **explore-path fallback** (which can navigate through `Unmapped` space) is **never tried** when the drone is stuck at an unmapped start.

### 5. Rescan progress gate kills retries

`can_retry` requires `last_scan_made_progress == true` for passes beyond 0. If the second scan pass maps zero new cells (same beams, same walls), `last_scan_made_progress = false` and the algorithm quits — even though most of the map is still `Unmapped`.

### 6. `latest_scan` is currently ignored

`MappingAlgorithmImpl::nextStep` receives `latest_scan` but does `(void)latest_scan`. Ex1 used scan results to decide on next actions. This may be fine if the algorithm reads the output map instead, but verify intentionally.

### 7. Validate on the actual scenario maps, not just CI

The integration tests only assert `mission_score >= 0`, which passes even on `-1` missions that stop at 6 mapped cells. **Always run**:

```bash
./build/drone_mapper_simulation scenarios/composition_scenario3.yaml /tmp/out3
grep mission_score /tmp/out3/simulation_output.yaml
./build/maps_comparison data_maps/scenario_3_map.npy /tmp/out3/output_results/run_0001/output_map.npy
```

A score of 0.55 on scenario 3 (99% unmapped) means the algorithm stopped at start. A working port should produce scores meaningfully above 0 and the drone should traverse the map.

### 8. Compare mapped cell count, not just status

After a run, check how many voxels were actually mapped in `output_map.npy`. The mission can report `COMPLETED` or `FinishedWithUnmappableVoxels` and still have mapped only a handful of cells. Use the `temp/analyze_scenario3.py` script (or the same pattern) to inspect coverage.

---

## After each port chunk

1. Run component test filter for affected component.
2. Scan new code against `docs/review-error-codes.md` (e03, e07, e08, e16, e17, e22).
3. Profile small-map runtime if algorithm changed (b05).
4. **Run scenario 3 manually and check score and mapped cell count** — do not rely on CI alone.

## Full mapping table

See `refactor_by_component.md`.
