---
name: debug-simulation-failure
description: Diagnoses Drone-Mapper-ex2 scenario failures, timeouts (b05), score -1, and error logs. Use when a simulation run fails, hangs, or produces unexpected scores.
---

# Debug Simulation Failure

## Gather evidence

1. Find the run folder: `output_results/run_NNNN/` (`output_map.npy`, `error.log`).
2. Match `run_id` in `simulation_output.yaml` to folder (`run_0001` ↔ `run_id: 1`).
3. Check `mission_score`, `mission_results[].status`, `mission_results[].errors`.
4. Reproduce with minimal composition YAML (single simulation × single mission × single drone × single lidar).

## Trace call stack

```
SimulationManager
  → SimulationRunImpl::run()
    → MissionControlImpl::runMission()
      → DroneControlImpl::step() [loop]
        → MappingAlgorithmImpl::nextStep()
        → MockMovement / MockLidar
        → ScanResultToVoxels → output_map.set()
    → MapsComparison::compare()
```

## Common causes

| Symptom | Check |
|---------|-------|
| `score: -1` | Error log for immediate entry; collision, boundary, parse failure |
| `max_steps` | Algorithm not converging; frontier stuck; increase efficiency |
| Timeout (b05) | Redundant rescans; O(n²) loops in `nextStep`; profile step count |
| Wrong score | `MapConfig` offset/resolution mismatch; comparison bounds |
| Missing file (b06) | Path resolution (CWD vs relative vs absolute) |

## Timeout mitigation

- Reduce rescan of already-mapped cells
- Cap work per `nextStep` call
- Verify `max_steps` from mission config is respected
- Test on smallest map first

## Verify logging

Errors must appear in error log **at occurrence time**, not at end of run. If the log is empty until the composition finishes, the ex1 deferred-flush pattern was likely ported by mistake.

Check that a failed run still allows later runs in `simulation_output.yaml` (ex1-style `return 1` from main would prevent this).

## Run single component

```bash
./build/drone_mapper_simulation_test --gtest_filter=DroneControl.*
./build/drone_mapper_simulation_test --gtest_filter=MappingAlgorithm.*
```

## maps_comparison isolation

```bash
./build/maps_comparison path/to/hidden.npy path/to/output.npy
```

Compare score to `simulation_output.yaml` entry.
