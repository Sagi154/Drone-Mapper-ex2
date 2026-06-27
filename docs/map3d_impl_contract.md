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
- `save(path)` writes the backing `NpyArray` via TinyNPY `SaveNPY`.

## Reference map (canonical format)

`data_maps/benchmark_map.npy` (ClassicCube-2, shape `(29, 30, 31)`, `uint8`) is the instructor-provided reference map from the skeleton. All synthetic test maps must use the same on-disk format:

- dtype: `uint8` (hidden/input maps) or `int8` (mutable output maps)
- shape: 3D `(nx, ny, nz)` — x=dim0, y=dim1, z=dim2, C-order
- values: `0` = Empty, `1` = Occupied for student-authored fixtures

The benchmark map may also contain ClassicCube export type codes (`2`, `3`, `4`, …). `Map3DImpl` treats only `0`/`1` as Empty/Occupied; other stored bytes map to `Unmapped`. Use `sim_benchmark.yaml` for integration scenarios on this map.

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
