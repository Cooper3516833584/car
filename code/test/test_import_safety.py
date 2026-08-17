from pathlib import Path
import os
import subprocess
import sys
import unittest


class ImportSafetyTests(unittest.TestCase):
    def test_main_and_new_components_import_without_starting_threads(self):
        code_root = Path(__file__).resolve().parents[1]
        script = (
            "import threading;"
            "before=tuple(t.name for t in threading.enumerate());"
            "import components.competition_track;"
            "import components.fixed_track_runtime;"
            "import components.radar_camera_line_following;"
            "after=tuple(t.name for t in threading.enumerate());"
            "assert before == after == ('MainThread',), (before, after)"
        )
        environment = os.environ.copy()
        environment["PYTHONPATH"] = str(code_root)
        result = subprocess.run(
            [sys.executable, "-c", script],
            cwd=code_root,
            env=environment,
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
