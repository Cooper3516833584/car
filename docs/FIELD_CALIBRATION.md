# Field calibration

Do not invent field dimensions, obstacles, radar extrinsics or initial pose. Copy `code/ros2_ws/src/car_nav_bringup/config/field.example.yaml`, then enter measured `boundary_m` vertices counter-clockwise and explicit fixed obstacle polygons.

Measure D500 `x`, `y`, and yaw from the rear-axle-centre `base_link` origin. Use metres and REP-103 yaw (counter-clockwise positive) in ROS launch arguments. The D500 component internally converts to its centimetre/clockwise convention at this boundary.

Before enabling HC-14, establish an `/initialpose` in the fixed field map and confirm `map -> odom`, `odom -> base_link`, and `base_link -> laser`. The mission bridge refuses goals outside the measured polygon or while localization is not ready.
