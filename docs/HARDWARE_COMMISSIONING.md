# Hardware commissioning

Only one process may own the C10B and steering servo. `ackermann_base_node` takes `/run/lock/car-hardware.lock`; stop the legacy `code/main.py` before launching ROS.

First use `use_hardware:=false dry_run_base:=true`. Then test on a raised chassis at at most `0.05 m/s` forward and `0.03 m/s` reverse: straight, both turn directions, reverse turns, a direction change, command timeout, localization loss, emergency stop, and SIGINT. Confirm every stop centres the servo.

HC-14 remains disabled unless `enable_hc14:=true` and `GROUND_STATION_HMAC_KEY_HEX` is present. Do not change HC-14 AT settings during commissioning.
