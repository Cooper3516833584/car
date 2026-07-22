# ROS 2 Nav2 integration

The original `code/main.py` and Python `Navigation` implementation remain the rollback path.  ROS nodes only adapt existing hardware components.

The TF chain is exactly `map -> odom -> base_link -> laser`. D500 ICP publishes continuous `odom -> base_link`; wall-line corrections use `GlobalCorrectionMode.UPDATE_ALIGNMENT`, so they update `map -> odom` only.

Nav2 uses Smac Hybrid-A* with `REEDS_SHEPP`, MPPI `Ackermann` motion model and a conservative `0.49 m` turning radius. Both supplied behavior trees exclude `Spin`; the base node rejects any in-place Twist and centres/stops on timeout, stale localization or emergency stop.

The Smac plugin is written using the current `nav2_smac_planner::SmacPlannerHybrid` class spelling; this is required for Jazzy, whereas older releases used the slash spelling. Collision Monitor uses Jazzy's `min_points` parameter (Humble requires the corresponding `max_points` semantics), so the board distribution must be checked before reuse.

Build on the board with `code/scripts/build_ros2.sh`. Generate a map only after supplying a measured field file:

```bash
ros2 run car_nav_bringup generate_field_map /home/radxa/car/config/field.yaml /home/radxa/car/maps/competition_field
```
