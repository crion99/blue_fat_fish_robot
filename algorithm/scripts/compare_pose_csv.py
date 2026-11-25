#!/usr/bin/env python3
"""Compare timestamped simulation positions in the same camera optical frame."""

import argparse
import csv
import math
from bisect import bisect_left
from collections import Counter
from pathlib import Path

AXES = ("x_right_m", "y_down_m", "z_forward_m")


def stamp_ns(row):
    return int(row["stamp_sec"]) * 1_000_000_000 + int(row["stamp_nanosec"])


def position(row):
    values = tuple(float(row[axis]) for axis in AXES)
    if not all(map(math.isfinite, values)):
        raise ValueError("position contains a non-finite value")
    return values


def read_rows(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("vision", type=Path)
    parser.add_argument("truth", type=Path, nargs="?")
    parser.add_argument("--max-offset-ms", type=float, default=10.0)
    args = parser.parse_args()
    if not math.isfinite(args.max_offset_ms) or args.max_offset_ms < 0:
        parser.error("--max-offset-ms must be finite and nonnegative")

    rows = read_rows(args.vision)
    counts = Counter(row["status"] for row in rows)
    print(f"Vision frames: {len(rows)} ({dict(counts)})")
    if rows:
        stamps = [stamp_ns(row) for row in rows]
        print(f"Vision time span: {(max(stamps) - min(stamps)) / 1e9:.3f} s")
    valid = [row for row in rows if row["status"] == "ok"]
    if valid:
        depths = [position(row)[2] for row in valid]
        print(f"Estimated depth range: {min(depths):.4f} to {max(depths):.4f} m")

    if args.truth is None:
        return

    truth_rows = read_rows(args.truth)
    by_time = {stamp_ns(row): position(row) for row in truth_rows}
    if len(by_time) != len(truth_rows):
        parser.error("truth has duplicate timestamps; use a single simulation run")
    timestamps = sorted(by_time)
    errors = []
    offsets = []
    for row in valid:
        time = stamp_ns(row)
        index = bisect_left(timestamps, time)
        nearby = timestamps[max(0, index - 1):index + 1]
        if not nearby:
            continue
        nearest = min(nearby, key=lambda sample: abs(sample - time))
        offset_ms = abs(nearest - time) / 1e6
        if offset_ms > args.max_offset_ms:
            continue
        errors.append(tuple(a - b for a, b in zip(position(row), by_time[nearest])))
        offsets.append(offset_ms)

    print(f"Matched: {len(errors)}/{len(valid)}; max allowed offset: {args.max_offset_ms:g} ms")
    if not errors:
        parser.exit(1, "No matching truth samples. Check simulation run and timestamps.\n")
    print(f"Largest matched offset: {max(offsets):.3f} ms")
    for axis, values in zip(AXES, zip(*errors)):
        bias = sum(values) / len(values)
        rmse = math.sqrt(sum(value ** 2 for value in values) / len(values))
        print(f"{axis}: bias={bias:.4f} m, RMSE={rmse:.4f} m")


if __name__ == "__main__":
    main()
