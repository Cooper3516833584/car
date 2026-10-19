# Testing Guide

> Status: documentation only. No test was executed for this change. The commands
> below are the project's documented entry points; review them before running.

## Test classes

| Class | Purpose | Where | Safe without hardware? |
|---|---|---|---|
| Unit tests (mock) | Pure logic: kinematics, protocol packing, config parsing, navigation math | `code/test/test_*.py` | Usually yes, if imports do not open devices |
| ROS 2 package tests | Bridge conversions and field geometry | `code/ros2_ws/src/*/test/` | Requires a ROS 2 environment |
| Hardware probe / calibration | Serial ports, servo sweep, LiDAR probe | `code/test/*probe*.py`, `*_test.py` on-vehicle scripts | **No — may move motors or servos** |
| Live mission | `main_task1.py` / `main_task2.py` | `code/` | **No** |

Some files named `*_test.py` under `code/test/` are on-vehicle scripts (for
example the HC-14 link test and the wheel test), **not** unit tests. Read the file
and confirm it does not touch `/dev/ttyACM*`, PWM or GPIO before executing.

## Documented commands

From the repository root, the unit suite uses `unittest`:

```bash
python3 -m unittest discover -s code/test -p "test_*.py"
```

Static checks:

```bash
python3 -m compileall -q code
git diff --check
```

ROS 2 packages, if used:

```bash
cd code/ros2_ws && colcon test
```

## What was not run here

- The unit suite: dependencies (`numpy`, `opencv`, `pyserial`) are not installed on
  this documentation host, and importing some modules may require hardware
  libraries. Marked **NOT RUN**, not "passing".
- Any on-vehicle script: **NOT RUN** by policy (could drive motors or servos).

## Before adding a test

- Put reusable logic tests in `code/test/` with `unittest`; do not open real
  devices at import time.
- Keep one-off vehicle scripts clearly named and documented; they are not part of
  automatic discovery.
- Never assert hardware success in a unit test; mock the transport instead.
