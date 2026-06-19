---
name: maps-comparison-utility
description: Implements or tests MapsComparison scoring and the maps_comparison standalone executable. Use when working on map scoring, comparison_config YAML, or stdout/stderr CLI contract.
---

# Maps Comparison Utility

## API

`MapsComparison::compare(origin, targets)` — standalone class, no interface required.

## Executable

```bash
./build/maps_comparison <origin_map> <target_map> [comparison_config=<path>]
```

## CLI contract

| Outcome | stdout | stderr |
|---------|--------|--------|
| Success | `93.5` (float only, no text) | empty |
| Error | `-1` | descriptive message |

## Scoring requirements

- Identical maps → **100**
- Very similar → close to 100, **not** exactly 100
- Very distinct → close to **0**
- Intermediate → reasonable gradient

## comparison_config YAML (optional)

When omitted: assume same offset, boundaries, resolution.

When provided: per-map `map_res_cm`, `map_offset`, `map_boundaries` for original and target.

## Implementation notes

- Use `IMap3D` / `Map3DImpl` — strong types for bounds and resolution.
- Different-resolution comparison is **bonus only** — not required for mandatory pass.
- Port scoring **ideas** from ex1 `Scorer`, not ex1 text-map format.

## Tests

Suite prefix: `MapsComparison.*`

```bash
./build/drone_mapper_simulation_test --gtest_filter=MapsComparison.*
```

Cases: identical, near-identical, distinct, invalid path, mismatched config.

## Error codes

Use structured errors in simulation; maps_comparison uses stderr message + stdout -1.
