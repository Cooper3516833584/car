#!/usr/bin/env python3
"""Evaluate the 30-degree straight-ahead camera profile without car hardware."""

from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path
import sys

import cv2

CODE_ROOT = Path(__file__).resolve().parents[1]
if str(CODE_ROOT) not in sys.path:
    sys.path.insert(0, str(CODE_ROOT))

from components.camera_line_follower import (
    BlackLineDetector,
    LineVisionConfig,
    PerspectiveConfig,
)


def vision_config() -> LineVisionConfig:
    return LineVisionConfig(
        perspective=PerspectiveConfig(
            source_points_norm=(
                (0.100, 0.950),
                (0.900, 0.950),
                (0.556, 0.100),
                (0.413, 0.100),
            ),
            output_width_px=200,
            output_height_px=250,
            ground_width_cm=80.0,
            ground_depth_cm=120.0,
        ),
        require_adaptive_confirmation=False,
        scan_near_cm=18.0,
        scan_far_cm=105.0,
        minimum_band_fill_ratio=0.20,
        use_expected_width_window=True,
        expected_line_width_cm=20.0,
        minimum_line_width_cm=7.0,
        maximum_line_width_cm=34.0,
        maximum_line_internal_gap_cm=8.0,
        maximum_center_jump_cm=18.0,
        morphology_close_size=9,
        polynomial_smoothing_alpha=0.32,
        transverse_stop_max_forward_cm=105.0,
        transverse_stop_max_height_cm=8.0,
        round_marker_min_height_cm=12.0,
        continuity_weight=0.12,
    )


def main() -> int:
    input_dir = Path(sys.argv[1])
    output_dir = Path(sys.argv[2])
    output_dir.mkdir(parents=True, exist_ok=True)
    detector = BlackLineDetector(vision_config())
    observations = {}
    for source in sorted(input_dir.glob("raw-*.jpg")):
        frame = cv2.imread(str(source))
        if frame is None:
            raise RuntimeError(f"cannot read {source}")
        observation, debug = detector.process(frame, return_debug=True)
        observations[source.name] = asdict(observation)
        cv2.imwrite(str(output_dir / source.name.replace("raw-", "debug-")), debug)
    print(json.dumps(observations, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
