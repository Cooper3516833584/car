from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from car_nav_bringup.generate_field_map import generate  # noqa: E402


class FieldMapGenerationTests(unittest.TestCase):
    def test_generates_map_only_from_explicit_vertices(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            field = root / "field.yaml"
            field.write_text("boundary_m: [[0, 0], [2, 0], [2, 1], [0, 1]]\nobstacle_polygons_m: []\n", encoding="utf-8")
            pgm, metadata = generate(field, root / "map", resolution_m=0.1)
            self.assertTrue(pgm.exists()); self.assertTrue(metadata.exists())
            self.assertIn("resolution: 0.1", metadata.read_text(encoding="utf-8"))


if __name__ == "__main__": unittest.main()
