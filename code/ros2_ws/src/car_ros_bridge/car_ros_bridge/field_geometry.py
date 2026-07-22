"""Static competition field validation and occupancy-map rasterisation."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


Point = tuple[float, float]


def point_in_polygon(point: Point, polygon: Iterable[Point]) -> bool:
    points = tuple(polygon)
    if len(points) < 3:
        return False
    x, y = point
    inside = False
    for index, (x1, y1) in enumerate(points):
        x2, y2 = points[(index + 1) % len(points)]
        if (y1 > y) != (y2 > y):
            crossing = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < crossing:
                inside = not inside
    return inside


@dataclass(frozen=True, slots=True)
class FieldGeometry:
    boundary_m: tuple[Point, ...]
    obstacle_polygons_m: tuple[tuple[Point, ...], ...] = ()
    boundary_margin_m: float = 0.0

    def __post_init__(self) -> None:
        if len(self.boundary_m) < 3:
            raise ValueError("field boundary requires at least three measured vertices")
        if self.boundary_margin_m < 0.0:
            raise ValueError("boundary_margin_m must be non-negative")

    def contains_goal(self, x_m: float, y_m: float) -> bool:
        return point_in_polygon((x_m, y_m), self.boundary_m) and not any(
            point_in_polygon((x_m, y_m), obstacle) for obstacle in self.obstacle_polygons_m
        )

    def occupancy(self, *, resolution_m: float) -> tuple[int, int, float, float, list[int]]:
        if resolution_m <= 0.0:
            raise ValueError("resolution_m must be positive")
        xs, ys = zip(*self.boundary_m)
        origin_x, origin_y = min(xs), min(ys)
        width = int((max(xs) - origin_x) / resolution_m) + 1
        height = int((max(ys) - origin_y) / resolution_m) + 1
        data: list[int] = []
        for row in range(height):
            for col in range(width):
                point = (origin_x + (col + 0.5) * resolution_m, origin_y + (row + 0.5) * resolution_m)
                data.append(0 if self.contains_goal(*point) else 100)
        return width, height, origin_x, origin_y, data

    @classmethod
    def from_yaml(cls, path: str | Path) -> "FieldGeometry":
        try:
            import yaml
        except ModuleNotFoundError as exc:
            raise RuntimeError("PyYAML is required to read field configuration") from exc
        raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        if not isinstance(raw, dict) or "boundary_m" not in raw:
            raise ValueError("field config requires measured boundary_m vertices")
        boundary = tuple(tuple(map(float, point)) for point in raw["boundary_m"])
        obstacles = tuple(tuple(tuple(map(float, point)) for point in polygon) for polygon in raw.get("obstacle_polygons_m", ()))
        return cls(boundary, obstacles, float(raw.get("boundary_margin_m", 0.0)))
