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

## Tests

```bash
./build/drone_mapper_simulation_test --gtest_filter=Map3DImpl.*
```
