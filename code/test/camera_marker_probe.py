#!/usr/bin/env python3
"""Print wide-marker geometry from one camera frame without motor access."""

from __future__ import annotations

from pathlib import Path
import sys

import cv2
import numpy as np

CODE_ROOT = Path(__file__).resolve().parents[1]
if str(CODE_ROOT) not in sys.path:
    sys.path.insert(0, str(CODE_ROOT))

from components.camera_line_follower import BlackLineDetector
from components.radar_camera_line_following import RadarCameraLineApplication


def main() -> int:
    config = RadarCameraLineApplication._front_camera_vision_config()
    detector = BlackLineDetector(config)
    capture = cv2.VideoCapture(0, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, config.frame_width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config.frame_height)
    capture.set(cv2.CAP_PROP_FPS, config.camera_fps)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*config.fourcc))
    try:
        for _ in range(config.warmup_frames):
            capture.grab()
        ok, frame = capture.read()
        if not ok or frame is None:
            raise RuntimeError("camera read failed")
    finally:
        capture.release()

    homography = detector._get_homography(frame.shape[1], frame.shape[0])
    perspective = config.perspective
    bird = cv2.warpPerspective(
        frame,
        homography,
        (perspective.output_width_px, perspective.output_height_px),
    )
    mask, _, _ = detector._segment_black_line(bird)
    height_px, width_px = mask.shape
    x_scale = perspective.ground_width_cm / width_px
    y_scale = perspective.ground_depth_cm / height_px
    rows = []
    for row_index, row in enumerate(mask):
        longest = max(
            (end - start for start, end in detector._true_runs(row > 0)),
            default=0,
        )
        width_cm = longest * x_scale
        if width_cm >= config.transverse_stop_min_width_cm:
            rows.append(
                (
                    perspective.ground_depth_cm - row_index * y_scale,
                    width_cm,
                )
            )
    print(rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
