from __future__ import annotations

from pathlib import Path
import unittest


class BehaviorTreeSafetyTests(unittest.TestCase):
    def test_ackermann_trees_do_not_request_spin(self) -> None:
        trees = Path(__file__).resolve().parents[1] / "behavior_trees"
        content = "\n".join(path.read_text(encoding="utf-8") for path in trees.glob("*.xml"))
        self.assertNotIn("Spin", content)


if __name__ == "__main__": unittest.main()
