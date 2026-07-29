"""Read-only adapter from an atomic car runtime snapshot to FleetBus units."""

import time

from .models import CarFleetState, CarOperationState, NodeFlags


_BUSY_STATES = {"PLANNING", "FOLLOWING", "FINAL_APPROACH", "GEAR_CHANGE"}


class CarFleetStateProvider:
    def __init__(self, runtime_snapshot_provider, started_at=None):
        self._runtime_snapshot_provider = runtime_snapshot_provider
        self._started_at = (
            time.monotonic() if started_at is None else float(started_at)
        )

    def __call__(self) -> CarFleetState:
        now = time.monotonic()
        snapshot = self._runtime_snapshot_provider()
        pose = snapshot.pose
        timeout_s = float(getattr(snapshot, "localization_timeout_s", 0.5))
        pose_fresh = pose is not None and now - pose.timestamp_s <= timeout_s
        pose_valid = bool(snapshot.ready and pose_fresh)
        state_name = getattr(snapshot.navigation_state, "name", "")
        flags = 0
        if snapshot.ready:
            flags |= int(NodeFlags.READY)
        if snapshot.map_ready:
            flags |= int(NodeFlags.MAP_READY)
        if pose_valid:
            flags |= int(NodeFlags.POSE_VALID)
        if state_name in _BUSY_STATES:
            flags |= int(NodeFlags.BUSY)
        degraded = bool(
            snapshot.ready and (not pose_valid or snapshot.localization_degraded)
        )
        if degraded:
            flags |= int(NodeFlags.LOCALIZATION_DEGRADED)

        if pose is None:
            x_cm = y_cm = heading_cdeg = 0
        else:
            x_cm = round(pose.x_cm)
            y_cm = round(pose.y_cm)
            heading_cdeg = round(pose.heading_deg * 100.0) % 36000
        quality = 4 if pose_valid else (2 if pose is not None else 0)
        return CarFleetState(
            node_flags=flags,
            node_uptime_ms=max(0, round((now - self._started_at) * 1000.0)),
            x_cm=x_cm,
            y_cm=y_cm,
            heading_cdeg=heading_cdeg,
            operation_state=int(self._operation_state(state_name, snapshot.ready)),
            pose_quality=quality,
            error_code=int(snapshot.error_code),
        )

    @staticmethod
    def _operation_state(state_name, ready):
        mapping = {
            "PLANNING": CarOperationState.PLANNING,
            "FOLLOWING": CarOperationState.FOLLOWING,
            "FINAL_APPROACH": CarOperationState.FINAL_APPROACH,
            "GEAR_CHANGE": CarOperationState.GEAR_CHANGE,
            "ARRIVED": CarOperationState.ARRIVED,
            "PAUSED": CarOperationState.PAUSED,
            "BLOCKED": CarOperationState.BLOCKED,
            "LOCALIZATION_LOST": CarOperationState.LOCALIZATION_LOST,
            "FAILED": CarOperationState.FAILED,
            "CLOSED": CarOperationState.CLOSED,
        }
        if state_name in mapping:
            return mapping[state_name]
        return CarOperationState.READY if ready else CarOperationState.STARTING
