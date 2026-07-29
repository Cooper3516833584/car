#!/usr/bin/env python3
"""Capture camera-only line-detector diagnostics without touching car hardware."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time

import cv2

CODE_ROOT = Path(__file__).resolve().parents[1]
if str(CODE_ROOT) not in sys.path:
    sys.path.insert(0, str(CODE_ROOT))

from components.camera_line_follower import BlackLineDetector
from main_radar_camera_line_following import RadarCameraLineApplication


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--output-dir", default="/tmp/camera-line-snapshot")
    parser.add_argument("--frames", type=int, default=12)
    args = parser.parse_args()
    if args.frames <= 0:
        parser.error("--frames must be positive")

    config = RadarCameraLineApplication._front_camera_vision_config()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    capture = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    if not capture.isOpened():
        raise RuntimeError(f"cannot open camera {args.camera}")
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, config.frame_width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config.frame_height)
    capture.set(cv2.CAP_PROP_FPS, config.camera_fps)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*config.fourcc))
    negotiated = {
        "width": int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
        "height": int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        "fps": capture.get(cv2.CAP_PROP_FPS),
    }
    detector = BlackLineDetector(config)
    observations = []
    try:
        for _ in range(config.warmup_frames):
            capture.grab()
        for index in range(args.frames):
            ok, frame = capture.read()
            if not ok or frame is None:
                raise RuntimeError(f"camera read failed at frame {index}")
            observation, debug = detector.process(
                frame,
                timestamp_s=time.monotonic(),
                return_debug=True,
            )
            observations.append(asdict(observation))
            cv2.imwrite(str(output_dir / f"raw-{index:02d}.jpg"), frame)
            cv2.imwrite(str(output_dir / f"debug-{index:02d}.jpg"), debug)
    finally:
        capture.release()

    metadata = {
        "requested": {
            "width": config.frame_width,
            "height": config.frame_height,
            "fps": config.camera_fps,
            "fourcc": config.fourcc,
        },
        "negotiated": negotiated,
        "observations": observations,
    }
    metadata_path = output_dir / "observations.json"
    metadata_path.write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    print(metadata_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
