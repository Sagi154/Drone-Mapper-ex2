---
name: implement-ex2-component
description: Implements or wires a single Assignment 2 skeleton component with tests. Use when implementing SimulationManager, DroneControlImpl, Map3DImpl, YAML parsing, factory wiring, or any ex2 component from the course skeleton.
---

# Implement Ex2 Component

## Before coding

1. Read the interface in `include/drone_mapper/` and stub in `src/`.
2. Read the matching flow in `docs/HLD.md` (when copied from skeleton).
3. Check `docs/ex1-to-ex2-porting.md` for ex1 reuse hints — **ideas only**, not copy-paste.
4. Read `docs/ex1-mistakes.md` — avoid listed anti-patterns.
5. For output paths / YAML shape: read **`readme.txt`** (OUTPUT LAYOUT section).

## Implementation steps

1. Implement **only** the target component; inject dependencies via constructor.
2. Use skeleton types from `types/` and strong types from `Units.h`.
3. Do **not** modify interfaces marked "Do not change".
4. Free helpers → anonymous namespace in `.cpp`.
5. Mark non-mutating methods `const` (e07).

## After coding

1. Add/update tests in `tests/components/` with correct GTest suite prefix.
2. Run: `./build/drone_mapper_simulation_test --gtest_filter=<Component>.*`
3. If wiring changed, update `docs/HLD.md`.

## Component-specific notes

| Component | Key behavior |
|-----------|--------------|
| `DroneControlImpl` | movement → scan → ScanResultToVoxels → set |
| `MappingAlgorithmImpl` | `nextStep(state, nullptr)` on first call — **see algorithm contract below** |
| `MissionControlImpl` | loop `drone_control.step()` until done/max_steps/error |
| `SimulationRunFactoryImpl` | wire full object graph; set paths via `runOutputMap` / `runErrorLog` from `src/io/`; pass `run_id` from manager |
| `SimulationManager` | aligned (simulation, mission) pairs × drones × lidars; compute `run_id`; aggregate `SimulationManagerReport`; write `simulation_output.yaml` |
| `Map3DImpl` | `.npy` load/save; `MapConfig` boundaries/offset/resolution; hidden `uint8` read clamp (`>= 1` → Occupied); output `int8` full enum |

## MappingAlgorithmImpl — contract that must hold

The skeleton's step/`nextStep` model means one scan orientation is emitted per step. The algorithm is correct only if a full scan pass (all orientations) actually clears a **passable local bubble** before the first planning phase. Without that, `isSpherePassable` blocks movement from step 1.

**Non-negotiable checks before shipping:**

1. After the first scan pass completes, `isSpherePassable` at the start position must return `true` — otherwise the drone is permanently stuck.
2. `drone.radius < grid_resolution` → footprint stamps only the center cell → **neighbors are not cleared by footprint**. The scan pass must do the work instead.
3. Coarse pass orientation count (depends on `el_step_base` × `kCoarsePassAngularScale`) can be as low as ~6 for common configs. Verify it's enough to cover local space.
4. `MappingAlgorithmFrontier::diagnose` bails immediately if start is not passable → explore/unstick fallback never runs. Fix passability first.
5. `last_scan_made_progress` gates rescan retries — if pass 1 maps 0 new cells, the algorithm quits even with 99% of the map unknown.
6. Validate by running an instructor focused composition and checking that `mission_score >= 90`:
   `./build/drone_mapper_simulation tests/data/instructor/compositions/composition_small_room.yaml /tmp/out`

See `port-from-ex1` skill for the full behavioral checklist.

## Authority

Assignment 2 + skeleton APIs > Review Guideline > ex1 reference.
