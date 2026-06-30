# Map3DImpl behavior contract

Implemented in `src/Map3DImpl.cpp`. Used by `MockLidar`, `ScanResultToVoxels`, `MapsComparison`, and factory wiring.

## World to voxel mapping

For world position `pos` and `MapConfig { offset, resolution }`:

```
index_axis = (pos_axis - offset_axis) / resolution
```

- Indices must be non-negative integers aligned to the grid (fractional positions are out of bounds).
- Array layout is NumPy C-order: `linear = ix * (ny * nz) + iy * nz + iz` for shape `(nx, ny, nz)`.
- World axes map to array dimensions as `(x -> 0, y -> 1, z -> 2)`.

Example: `single_voxel_x2_y4_z2.npy` with resolution `10 cm`, offset `(0,0,0)` stores the occupied cell at world `(20, 40, 20) cm`.

## Bounds

`isInBounds(pos)` is true when:

1. `pos` is inside `MapConfig.boundaries` (inclusive min/max on each axis), and
2. the computed voxel index lies within the `.npy` array shape.

If `MappingBounds` are unset (all zero), they are derived from array shape, offset, and resolution.

## Query / mutation semantics

| Method | Out of bounds | In bounds |
|--------|---------------|-----------|
| `atVoxel` | `VoxelOccupancy::OutOfBounds` | stored occupancy |
| `set` | no-op | writes occupancy |
| `isInBounds` | `false` | `true` |

## Storage / I/O

- Hidden maps load as `uint8` (`0` = Empty, `1` = Occupied).
- Mutable output maps may use `int8` to store full `VoxelOccupancy` values.
- On read, any stored value `>= 1` is treated as `Occupied` (see clamp rule below).
- `save(path)` writes the backing `NpyArray` via TinyNPY `SaveNPY`.

## Reference map (canonical format)

All hidden-map `.npy` files use the same on-disk format:

- dtype: `uint8` (hidden/input maps) or `int8` (mutable output maps)
- shape: 3D `(nx, ny, nz)` — x=dim0, y=dim1, z=dim2, C-order
- values: `0` = Empty, `1` = Occupied for student-authored fixtures

`Map3DImpl` treats `0` as Empty and any value `>= 1` as Occupied — instructor-provided maps may encode extra information (e.g. `2`, `18`, `45`) in occupied cells; per course staff clarification, any such value still means Occupied. Unrecognized negative values map to `Unmapped`.

## Instructor integration maps

Vendored in `tests/data/instructor/map/`. Use these for integration tests and CI — not legacy `data_maps/benchmark_map.npy`.

| File | Shape | Resolution | Notes |
|------|-------|------------|-------|
| `scenario_house.npy` | `(29, 30, 31)` | 10 cm/voxel | `house_simulation.yaml`; `map_axes_offset.height_offset: 150` |
| `scenario_small.npy` | `(20, 20, 20)` | 10 cm/voxel | `small_simulation_room.yaml`, `small_simulation_out.yaml` |
| `scenario_big.npy` | `(30, 30, 30)` | 10 cm/voxel | `large_simulation_room.yaml`, `large_simulation_out.yaml` |

- Full composition: `tests/data/instructor/sim_compose.yaml` (24 runs when expanded)
- Focused compositions: `tests/data/instructor/compositions/composition_{small_room,big_room,house_lower}.yaml`
- Integration tests: `Integration.Instructor*` in `tests/integration/test_integration_full_flow.cpp`
- CI: `.github/workflows/ci.yml` — **Check instructor composition mission scores** (asserts `mission_score >= 90`)

## Dev/unit-test maps (`data_maps/`)

Small golden maps (`single_voxel_*.npy`, etc.) and `benchmark_map.npy` for component tests (`Map3DImpl.*`, `MapsComparison.*`). Config fixture: `tests/data/configs/sim_benchmark.yaml`; path helper: `test_support::benchmarkMapPath()`.

When writing `.npy` test fixtures in Python/NumPy:

```python
import numpy as np
arr = np.zeros((nx, ny, nz), dtype=np.uint8)
arr[ix, iy, iz] = 1   # mark occupied voxels
np.save("my_test_map.npy", arr)
```

## Tests

```bash
./build/drone_mapper_simulation_test --gtest_filter=Map3DImpl.*
```
