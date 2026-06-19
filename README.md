# Drone-Mapper-ex2

TAU Advanced Topics in Programming — Assignment 2 (2026B).

Course skeleton merged into this repo. Stubs compile; full simulator, YAML I/O, tests, and algorithm are **to be implemented**. See `workplan.md` for execution plan and `AGENTS.md` for Cursor agent guidance.

## Contributors

| Name | Student ID |
|------|------------|
| Sagi Eisenberg | 207190406 |
| Yoav Naaman | 209543255 |

## Project layout

```text
include/drone_mapper/     Interfaces, types, component headers
src/                      Implementations and executable entry points
data_maps/                Example .npy voxel maps
cpp_yaml_example/         yaml-cpp reference (course example)
tests/                    GTest suites (to be added)
docs/                     HLD, checklists, ex1 lessons
context/                  Original assignment docx files
.cursor/                  Agent rules and skills
```

## Build

Requires [vcpkg](https://vcpkg.io/) with `VCPKG_ROOT` set.

```bash
cmake --preset default
cmake --build --preset default
```

Targets: `drone_mapper_simulation`, `maps_comparison`, `example_yml`.  
`drone_mapper_simulation_test` — add when test CMake is wired (see workplan Phase 4).

## Run (current skeleton)

```bash
./build/drone_mapper_simulation [<simulation.yaml>] [<output_path>]
./build/maps_comparison <origin_map> <target_map> [comparison_config=<path>]
```

## Docs

- `workplan.md` — team phases and gates
- `refactor_by_component.md` — ex1 → ex2 porting map
- `docs/assignment2-checklist.md` — submission requirements
- `docs/HLD.md` — design (keep aligned with code)
- `AGENTS.md` — Cursor agent entry point

## Ex1 reference

Algorithm ideas only: `../Drone-Mapper/`. Do **not** copy ex1 structure — see `docs/ex1-mistakes.md`.
