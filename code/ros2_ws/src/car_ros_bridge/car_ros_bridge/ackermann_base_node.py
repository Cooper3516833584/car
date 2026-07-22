"""Final /cmd_vel hardware bridge with Ackermann-only safety gates."""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
import time

from components import AckermannDrive, MotorDirection
from .cmd_vel_conversion import AckermannTarget, UnsafeTwist, twist_to_ackermann


@dataclass
class BaseCommandState:
    command_timeout_s: float = 0.25
    direction_change_stop_s: float = 0.25
    last_target: AckermannTarget | None = None
    last_command_time: float = 0.0
    stopped_until: float = 0.0
    emergency_stop: bool = False
    localization_ready: bool = False

    def accept(self, target: AckermannTarget, now: float) -> None:
        if self.last_target and target.speed_mm_s and self.last_target.speed_mm_s and target.forward != self.last_target.forward:
            self.stopped_until = now + self.direction_change_stop_s
        self.last_target, self.last_command_time = target, now

    def output(self, now: float) -> AckermannTarget:
        if self.emergency_stop or not self.localization_ready or now < self.stopped_until:
            return AckermannTarget(0.0, 0.0, True)
        if self.last_target is None or now - self.last_command_time > self.command_timeout_s:
            return AckermannTarget(0.0, 0.0, True)
        return self.last_target


def main() -> None:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_msgs.msg import Bool

    class AckermannBaseNode(Node):
        def __init__(self) -> None:
            super().__init__("ackermann_base_node")
            self.declare_parameter("dry_run", False)
            self.declare_parameter("max_speed_m_s", 0.30)
            self.declare_parameter("command_timeout_s", 0.25)
            self.declare_parameter("lock_path", "/run/lock/car-hardware.lock")
            self.state = BaseCommandState(command_timeout_s=float(self.get_parameter("command_timeout_s").value))
            self.max_speed_m_s = float(self.get_parameter("max_speed_m_s").value)
            self.drive = None
            if not self.get_parameter("dry_run").value:
                self.drive = AckermannDrive(hardware_lock_path=str(self.get_parameter("lock_path").value)).start()
            self.create_subscription(Twist, "/cmd_vel", self.on_twist, 10)
            self.create_subscription(Bool, "/car/localization_ready", self.on_ready, 1)
            self.create_subscription(Bool, "/car/emergency_stop", self.on_estop, 1)
            self.diagnostics = self.create_publisher(DiagnosticArray, "/car/diagnostics", 10)
            self.create_timer(0.05, self.tick)

        def on_twist(self, message: Twist) -> None:
            try:
                self.state.accept(twist_to_ackermann(message.linear.x, message.angular.z, max_speed_m_s=self.max_speed_m_s), time.monotonic())
            except UnsafeTwist as exc:
                self.get_logger().warning(str(exc))
                self.state.last_target = AckermannTarget(0.0, 0.0, True)

        def on_ready(self, message: Bool) -> None:
            self.state.localization_ready = message.data

        def on_estop(self, message: Bool) -> None:
            self.state.emergency_stop = message.data

        def tick(self) -> None:
            target = self.state.output(time.monotonic())
            if self.drive is not None:
                if target.speed_mm_s == 0.0:
                    self.drive.stop(center_steering=True)
                else:
                    self.drive.set_motion(target.speed_mm_s, target.steering_rad, direction=MotorDirection.FORWARD if target.forward else MotorDirection.REVERSE)
            status = DiagnosticStatus(level=DiagnosticStatus.OK, name="ackermann_base", message="stopped" if not target.speed_mm_s else "driving")
            status.values = [KeyValue(key="speed_mm_s", value=f"{target.speed_mm_s:.1f}"), KeyValue(key="steering_rad", value=f"{target.steering_rad:.4f}")]
            self.diagnostics.publish(DiagnosticArray(status=[status]))

        def destroy_node(self) -> bool:
            if self.drive is not None:
                self.drive.close()
            return super().destroy_node()

    rclpy.init()
    node = AckermannBaseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
