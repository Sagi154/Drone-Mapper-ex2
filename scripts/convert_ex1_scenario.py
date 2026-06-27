#!/usr/bin/env python3
"""Convert ex1 map_input.txt (sparse text) to ex2 hidden-map .npy (dense uint8)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


def parse_bounds(line: str) -> tuple[int, int, int, int, int, int]:
    tokens = line.split()
    if len(tokens) != 9 or tokens[0] != "bounds":
        raise ValueError(f"expected bounds line with 9 tokens, got: {line!r}")
    min_x, max_x = int(float(tokens[1])), int(float(tokens[2]))
    min_y, max_y = int(float(tokens[3])), int(float(tokens[4]))
    min_z, max_z = int(float(tokens[5])), int(float(tokens[6]))
    if min_x != 0 or min_y != 0 or min_z != 0:
        raise ValueError(f"unsupported non-zero min bounds: {line!r}")
    return min_x, max_x, min_y, max_y, min_z, max_z


def convert_map_input(input_path: Path) -> np.ndarray:
    bounds: tuple[int, int, int, int, int, int] | None = None
    occupied: list[tuple[int, int, int]] = []

    with input_path.open(encoding="utf-8") as handle:
        for line_num, raw in enumerate(handle, start=1):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            tokens = line.split()
            record = tokens[0]
            if record == "bounds":
                if bounds is not None:
                    raise ValueError(f"duplicate bounds at line {line_num}")
                bounds = parse_bounds(line)
            elif record == "occupied":
                if len(tokens) != 4:
                    raise ValueError(f"bad occupied record at line {line_num}: {line!r}")
                x, y, z = int(float(tokens[1])), int(float(tokens[2])), int(float(tokens[3]))
                occupied.append((x, y, z))
            elif record == "empty":
                continue
            else:
                raise ValueError(f"unknown record at line {line_num}: {record!r}")

    if bounds is None:
        raise ValueError(f"missing bounds line in {input_path}")

    _, max_x, _, max_y, _, max_z = bounds
    shape = (max_x + 1, max_y + 1, max_z + 1)
    arr = np.zeros(shape, dtype=np.uint8)

    for x, y, z in occupied:
        if not (0 <= x < shape[0] and 0 <= y < shape[1] and 0 <= z < shape[2]):
            raise ValueError(f"occupied ({x},{y},{z}) out of bounds for shape {shape}")
        arr[x, y, z] = 1

    return arr


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "scenario_dir",
        type=Path,
        help="ex1 scenario directory containing map_input.txt",
    )
    parser.add_argument(
        "output_npy",
        type=Path,
        help="destination .npy path",
    )
    args = parser.parse_args()

    input_path = args.scenario_dir / "map_input.txt"
    if not input_path.is_file():
        print(f"error: missing {input_path}", file=sys.stderr)
        return 1

    arr = convert_map_input(input_path)
    args.output_npy.parent.mkdir(parents=True, exist_ok=True)
    np.save(args.output_npy, arr)
    print(f"wrote {args.output_npy} shape={arr.shape} dtype={arr.dtype} occupied={int(arr.sum())}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
