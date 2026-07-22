"""HC-14 mission commands -> NavigateToPose action without ROS calls in the UART thread."""

from __future__ import annotations

from dataclasses import dataclass
import math
import queue

from components import (
    AckStatus, GroundNavigationProtocol, NavigationCommandReceipt,
    NavigationCommandRejected, NavigationGoal, RejectReason,
    SerialCommunicationDriver, load_navigation_hmac_key,
)
from .field_geometry import FieldGeometry


@dataclass(frozen=True, slots=True)
class PendingMission:
    goal: NavigationGoal
    receipt: NavigationCommandReceipt


class MissionBridgeLogic:
    """Thread-safe, ROS-free state gate for protocol callbacks and tests."""

    def __init__(self, field: FieldGeometry) -> None:
        self.field = field
        self.localization_ready = False
        self.active: PendingMission | None = None
        self.inbound: queue.Queue[PendingMission] = queue.Queue()

    def submit(self, goal: NavigationGoal, receipt: NavigationCommandReceipt) -> None:
        if not self.localization_ready:
            raise NavigationCommandRejected(RejectReason.LINK_DOWN, "localization is not ready")
        if self.active is not None:
            raise NavigationCommandRejected(RejectReason.TASK_BUSY, "a mission is already active")
        if not self.field.contains_goal(goal.x_cm / 100.0, goal.y_cm / 100.0):
            raise NavigationCommandRejected(RejectReason.BAD_PAYLOAD, "goal is outside measured field")
        mission = PendingMission(goal, receipt)
        self.active = mission
        self.inbound.put(mission)

    def finish(self) -> None:
        self.active = None


def main() -> None:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from std_msgs.msg import Bool

    class MissionBridgeNode(Node):
        def __init__(self) -> None:
            super().__init__("car_mission_bridge_node")
            self.declare_parameter("field_config")
            self.declare_parameter("enable_hc14", True)
            field_path = str(self.get_parameter("field_config").value)
            if not field_path:
                raise RuntimeError("field_config must name a measured field file")
            self.logic = MissionBridgeLogic(FieldGeometry.from_yaml(field_path))
            self.action = ActionClient(self, NavigateToPose, "/navigate_to_pose")
            self.estop_pub = self.create_publisher(Bool, "/car/emergency_stop", 1)
            self.current_goal_handle = None
            self.outbound: queue.Queue[bytes] = queue.Queue()
            self.protocol = GroundNavigationProtocol(key=load_navigation_hmac_key(), on_goal=self.logic.submit, on_stop=self.on_stop_from_serial)
            self.link = None
            if self.get_parameter("enable_hc14").value:
                self.link = SerialCommunicationDriver(on_bytes=self.on_serial_frame)
                self.link.start()
            self.create_subscription(Bool, "/car/localization_ready", self.on_ready, 1)
            self.create_timer(0.05, self.process_queues)

        def on_ready(self, message: Bool) -> None:
            self.logic.localization_ready = message.data

        def on_serial_frame(self, frame: bytes) -> None:
            try:
                for reply in self.protocol.handle_frame(frame):
                    self.outbound.put(reply)
            except Exception as exc:
                self.get_logger().warning(f"rejected serial frame: {exc}")

        def on_stop_from_serial(self, receipt: NavigationCommandReceipt) -> None:
            self.estop_pub.publish(Bool(data=True))
            if self.current_goal_handle is not None:
                self.current_goal_handle.cancel_goal_async()
            self.logic.finish()

        def process_queues(self) -> None:
            while self.link is not None and not self.outbound.empty():
                try: self.link.write(self.outbound.get_nowait())
                except Exception as exc: self.get_logger().error(f"HC-14 write failed: {exc}")
            if self.current_goal_handle is not None or self.logic.inbound.empty():
                return
            if not self.action.server_is_ready():
                self.get_logger().error("NavigateToPose action server is unavailable")
                return
            mission = self.logic.inbound.get_nowait()
            goal = NavigateToPose.Goal(); goal.pose = PoseStamped(); goal.pose.header.frame_id = "map"; goal.pose.header.stamp = self.get_clock().now().to_msg(); goal.pose.pose.position.x = mission.goal.x_cm / 100.0; goal.pose.pose.position.y = mission.goal.y_cm / 100.0
            heading = mission.goal.final_heading_deg
            if heading is None:
                heading = 0.0  # Nav2 position BT ignores orientation; keep a valid quaternion.
                goal.behavior_tree = "navigate_ackermann_position.xml"
            else:
                goal.behavior_tree = "navigate_ackermann_pose.xml"
            radians = math.radians(heading); goal.pose.pose.orientation.z = math.sin(radians / 2); goal.pose.pose.orientation.w = math.cos(radians / 2)
            future = self.action.send_goal_async(goal); future.add_done_callback(lambda result, mission=mission: self.goal_response(result, mission))

        def goal_response(self, future, mission: PendingMission) -> None:
            handle = future.result()
            if not handle.accepted:
                self.outbound.put(self.protocol.build_status_ack(mission.receipt, AckStatus.FAILED)); self.logic.finish(); return
            self.current_goal_handle = handle
            handle.get_result_async().add_done_callback(lambda result, mission=mission: self.goal_result(result, mission))

        def goal_result(self, future, mission: PendingMission) -> None:
            result = future.result()
            status = AckStatus.COMPLETED if result.status == 4 else AckStatus.FAILED
            self.outbound.put(self.protocol.build_status_ack(mission.receipt, status))
            self.current_goal_handle = None; self.logic.finish()

        def destroy_node(self) -> bool:
            self.estop_pub.publish(Bool(data=True))
            if self.link is not None: self.link.close()
            return super().destroy_node()

    rclpy.init(); node = MissionBridgeNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__": main()
