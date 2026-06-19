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
| `MappingAlgorithmImpl` | `nextStep(state, nullptr)` on first call |
| `MissionControlImpl` | loop `drone_control.step()` until done/max_steps/error |
| `SimulationRunFactoryImpl` | wire full object graph; transfer ownership to `SimulationRunImpl` |
| `SimulationManager` | cartesian product; aggregate `SimulationManagerReport` |
| `Map3DImpl` | `.npy` load/save; `MapConfig` boundaries/offset/resolution |

## Authority

Assignment 2 + skeleton APIs > Review Guideline > ex1 reference.
