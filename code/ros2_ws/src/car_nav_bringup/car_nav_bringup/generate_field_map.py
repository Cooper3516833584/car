"""Generate a Nav2 map only from explicitly measured field geometry."""

from __future__ import annotations

import argparse
from pathlib import Path

from car_ros_bridge.field_geometry import FieldGeometry


def generate(field_config: str | Path, output_prefix: str | Path, *, resolution_m: float = 0.02) -> tuple[Path, Path]:
    field = FieldGeometry.from_yaml(field_config)
    width, height, origin_x, origin_y, data = field.occupancy(resolution_m=resolution_m)
    prefix = Path(output_prefix); prefix.parent.mkdir(parents=True, exist_ok=True)
    pgm = prefix.with_suffix(".pgm"); yaml_path = prefix.with_suffix(".yaml")
    # Nav2 PGM: black occupied, white free.  Output is top-to-bottom.
    rows = [data[row * width:(row + 1) * width] for row in range(height)]
    pgm.write_bytes((f"P5\n{width} {height}\n255\n").encode() + bytes(0 if value else 254 for row in reversed(rows) for value in row))
    yaml_path.write_text("image: " + pgm.name + "\nresolution: " + str(resolution_m) + "\norigin: [" + str(origin_x) + ", " + str(origin_y) + ", 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n", encoding="utf-8")
    return pgm, yaml_path


def main() -> None:
    parser = argparse.ArgumentParser(); parser.add_argument("field_config"); parser.add_argument("output_prefix"); parser.add_argument("--resolution-m", type=float, default=0.02); args = parser.parse_args(); generate(args.field_config, args.output_prefix, resolution_m=args.resolution_m)


if __name__ == "__main__": main()
