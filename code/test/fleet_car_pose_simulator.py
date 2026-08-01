"""HC-14-only car pose simulator with no vehicle actuator access.

The simulator replies to ground-station FleetBus polls and trace requests with
a moving synthetic pose.  It imports no motor, steering, radar, or mission
entry point and exits after a bounded duration or a TARGETED_STOP command.
"""

import argparse
import math
import signal
import threading
import time


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--connect-hc14",
        action="store_true",
        help="open only the car CH340/HC-14 serial link",
    )
    parser.add_argument("--port", default=None)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--duration-s", type=float, default=120.0)
    parser.add_argument("--connect-timeout-s", type=float, default=5.0)
    return parser


class SimulatedCarStateProvider:
    """Generate a bounded circular FIELD-frame pose without touching hardware."""

    def __init__(self, state_type, node_flags):
        self._state_type = state_type
        self._node_flags = int(node_flags)
        self._started_at = time.monotonic()

    def __call__(self):
        elapsed_s = time.monotonic() - self._started_at
        angular_rate = -0.30
        angle = angular_rate * elapsed_s
        radius_cm = 55.0
        x_cm = 275.0 + radius_cm * math.cos(angle)
        y_cm = 300.0 + radius_cm * math.sin(angle)
        vx_cm_s = -radius_cm * angular_rate * math.sin(angle)
        vy_cm_s = radius_cm * angular_rate * math.cos(angle)
        return self._state_type(
            node_flags=self._node_flags,
            uptime_ms=round(elapsed_s * 1000.0) & 0xFFFFFFFF,
            x_cm=round(x_cm),
            y_cm=round(y_cm),
            heading_cdeg=round(math.degrees(angle - math.pi / 2.0) * 100.0)
            % 36000,
            vx_cm_s=round(vx_cm_s),
            vy_cm_s=round(vy_cm_s),
            battery_cV=1200,
            operation_state=0,
            pose_quality=100,
        )


def main():
    args = build_parser().parse_args()
    if not args.connect_hc14:
        print("No serial port opened. Add --connect-hc14 for the bounded test.")
        return 2
    if args.duration_s <= 0.0:
        raise SystemExit("--duration-s must be positive")
    if args.connect_timeout_s <= 0.0:
        raise SystemExit("--connect-timeout-s must be positive")

    from components.fleet_car_node import FleetCarNode
    from components.fleet_models import (
        AckReason,
        AckStatus,
        CarFleetState,
        CommandResult,
        NodeFlags,
    )
    from components.fleet_trace import TraceSamplingOptions
    from components.serial_communication import (
        DEFAULT_HC14_PORT,
        SerialCommunicationDriver,
    )

    stop_event = threading.Event()
    port = args.port or DEFAULT_HC14_PORT
    state_provider = SimulatedCarStateProvider(
        CarFleetState,
        NodeFlags.POSE_VALID | NodeFlags.READY | NodeFlags.COORDINATE_FRAME_SYNCED,
    )

    def reject_command(*_args):
        return CommandResult(
            AckStatus.REJECTED,
            AckReason.NOT_READY,
            "pose simulation active; actuator commands disabled",
        )

    def stop_simulator():
        stop_event.set()
        return CommandResult(AckStatus.COMPLETED)

    holder = {}
    link = SerialCommunicationDriver(
        port=port,
        baudrate=args.baudrate,
        on_bytes=lambda frame: holder["node"].feed_frame(frame),
    )
    node = FleetCarNode(
        writer=link.write,
        state_provider=state_provider,
        on_set_coordinate_frame=reject_command,
        on_navigate=reject_command,
        on_stop=stop_simulator,
        trace_options=TraceSamplingOptions(
            enabled=True,
            sample_interval_s=0.10,
            buffer_capacity=1800,
            min_distance_cm=0.5,
        ),
    )
    holder["node"] = node

    def request_stop(_signum, _frame):
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node.start()
    link.start()
    try:
        if not link.wait_connected(args.connect_timeout_s):
            raise RuntimeError(
                "car HC-14 did not connect within the timeout: {}".format(
                    link.last_error
                )
            )
        print(
            "Car pose simulator connected on {} at {} baud for {:.1f}s".format(
                port, args.baudrate, args.duration_s
            ),
            flush=True,
        )
        stop_event.wait(args.duration_s)
    finally:
        link.close()
        node.close()
        print(
            "Car pose simulator stopped; trace_samples={}".format(
                node.trace_buffer.recorded_samples
            ),
            flush=True,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
