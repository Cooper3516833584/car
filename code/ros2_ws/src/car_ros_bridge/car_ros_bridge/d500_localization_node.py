"""D500 adapter publishing /scan, continuous /odom and the required TF chain."""

from __future__ import annotations

import math

from components import D500RadarComponent, GlobalCorrectionMode, Pose2D, RadarMount
from .ros_conversions import radar_points_to_scan_ranges, radar_pose_to_ros, yaw_to_quaternion


def main() -> None:
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool
    from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

    class D500LocalizationNode(Node):
        def __init__(self) -> None:
            super().__init__("d500_localization_node")
            from rclpy.parameter import Parameter
            for name in ("radar_x_m", "radar_y_m", "radar_yaw_rad"):
                self.declare_parameter(name)
                if self.get_parameter(name).type_ == Parameter.Type.NOT_SET:
                    raise RuntimeError(f"required measured parameter {name} is missing")
            self.declare_parameter("use_hardware", True)
            mount = RadarMount(float(self.get_parameter("radar_x_m").value) * 100.0, float(self.get_parameter("radar_y_m").value) * 100.0, -math.degrees(float(self.get_parameter("radar_yaw_rad").value)))
            self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
            self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
            self.ready_pub = self.create_publisher(Bool, "/car/localization_ready", 1)
            self.tf = TransformBroadcaster(self)
            self.static_tf = StaticTransformBroadcaster(self)
            self.radar = D500RadarComponent(mount=mount, on_update=self.on_update, global_correction_mode=GlobalCorrectionMode.UPDATE_ALIGNMENT)
            self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.on_initial_pose, 1)
            self.publish_laser_tf(mount)
            if self.get_parameter("use_hardware").value:
                self.radar.start()

        def publish_laser_tf(self, mount: RadarMount) -> None:
            tf = TransformStamped(); tf.header.stamp = self.get_clock().now().to_msg(); tf.header.frame_id = "base_link"; tf.child_frame_id = "laser"
            tf.transform.translation.x = mount.x_forward_cm / 100.0; tf.transform.translation.y = mount.y_left_cm / 100.0
            q = yaw_to_quaternion(-math.radians(mount.yaw_cw_deg)); tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = q.x, q.y, q.z, q.w
            self.static_tf.sendTransform(tf)

        def on_initial_pose(self, message: PoseWithCovarianceStamped) -> None:
            q = message.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.radar.set_global_reference(self.radar.odometry.pose, Pose2D(message.pose.pose.position.x * 100.0, message.pose.pose.position.y * 100.0, -math.degrees(yaw)))

        def on_update(self, update) -> None:
            stamp = self.get_clock().now().to_msg(); scan = LaserScan(); scan.header.stamp = stamp; scan.header.frame_id = "laser"; scan.angle_min = -math.pi; scan.angle_max = math.pi; scan.angle_increment = 2 * math.pi / 720; scan.range_min = 0.02; scan.range_max = 12.0; scan.ranges = radar_points_to_scan_ranges(update.scan.points); self.scan_pub.publish(scan)
            pose = radar_pose_to_ros(update.odometry.pose); odom = Odometry(); odom.header.stamp = stamp; odom.header.frame_id = "odom"; odom.child_frame_id = "base_link"; odom.pose.pose.position.x = pose.x_m; odom.pose.pose.position.y = pose.y_m; q = yaw_to_quaternion(pose.yaw_rad); odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = q.x, q.y, q.z, q.w; self.odom_pub.publish(odom)
            odom_tf = TransformStamped(); odom_tf.header = odom.header; odom_tf.child_frame_id = "base_link"; odom_tf.transform.translation.x = pose.x_m; odom_tf.transform.translation.y = pose.y_m; odom_tf.transform.rotation = odom.pose.pose.orientation; self.tf.sendTransform(odom_tf)
            alignment = self.radar.get_alignment()
            if alignment is not None:
                map_pose = radar_pose_to_ros(alignment.pose_to_global(Pose2D())); map_tf = TransformStamped(); map_tf.header.stamp = stamp; map_tf.header.frame_id = "map"; map_tf.child_frame_id = "odom"; map_tf.transform.translation.x = map_pose.x_m; map_tf.transform.translation.y = map_pose.y_m; q = yaw_to_quaternion(map_pose.yaw_rad); map_tf.transform.rotation.x, map_tf.transform.rotation.y, map_tf.transform.rotation.z, map_tf.transform.rotation.w = q.x, q.y, q.z, q.w; self.tf.sendTransform(map_tf)
            self.ready_pub.publish(Bool(data=update.odometry.accepted and alignment is not None))

        def destroy_node(self) -> bool:
            self.radar.close(); return super().destroy_node()

    rclpy.init(); node = D500LocalizationNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__": main()
