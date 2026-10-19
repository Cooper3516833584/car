# Repository Structure

> Describes the existing layout. This change does not move or rename production
> modules.

```text
car/
├── code/
│   ├── main_task1.py            task 1 entry (loads TOML, delegates to core)
│   ├── main_task2.py            task 2 entry
│   ├── components/              reusable hardware and mission components
│   │   ├── radar_camera_line_following.py   shared mission core
│   │   ├── radar_driver.py      D500 scan parsing / pose
│   │   ├── camera_line_follower.py          lane error
│   │   ├── navigation.py, coordinate_navigation.py
│   │   ├── ackermann_drive.py   speed + steering facade
│   │   ├── rear_motor.py        C10B serial driver
│   │   ├── steering_servo.py    steering calibration + PWM
│   │   ├── fleet_car_node.py, fleet_trace.py
│   │   └── sound_light_alarm.py
│   ├── config/                  TOML models, loader, runtime state
│   ├── hal/                     GPIO / PWM abstraction
│   ├── fleet_bus/               FleetBus protocol and car node
│   ├── test/                    unit tests + on-vehicle diagnostic scripts
│   ├── ros2_ws/                 optional ROS 2 bridge and Nav2 bringup
│   └── former_code/             archived earlier implementation (read-only)
├── configs/                     vehicle TOML profiles
├── deploy/                      systemd templates
├── docs/                        configuration, calibration, architecture, protocol
└── tools/                       auxiliary scripts (if present)
```

## Entry points

| Purpose | Command |
|---|---|
| Task 1 mission | `python3 code/main_task1.py --config configs/<profile>.toml` |
| Task 2 mission | `python3 code/main_task2.py --config configs/<profile>.toml` |
| ROS 2 bringup | `code/ros2_ws/src/car_nav_bringup/launch/*.launch.py` |

## Where to add code

- Hardware driver or reusable component → `code/components/`
- Board-specific GPIO/PWM backend → `code/hal/`
- Configuration schema → `code/config/`
- Unit tests and one-off vehicle scripts → `code/test/`
- Long-form documentation → `docs/`

Do not add a new top-level mission entry without updating this map and the README.

## Generated / ignored

`__pycache__/`, `*.pyc`, `.venv/`, `logs/`, `runtime/`,
`code/radar_center_config.json`, and test temporary directories are ignored by
`.gitignore`. Build outputs under `code/ros2_ws/` are produced by `colcon` and are
not tracked.

## Related documents

- `docs/SYSTEM_ARCHITECTURE.md`
- `docs/DRIVE_PROTOCOL.md`
- `docs/CONFIGURATION.md`, `docs/CALIBRATION.md`
- `docs/TESTING.md`
