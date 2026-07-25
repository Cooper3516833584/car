#!/usr/bin/env python3
"""3x5 wildfire rescue entry point built on the production car navigation stack."""

from __future__ import annotations

import logging
import math
from pathlib import Path
import signal
import sys
import threading
import time

from components.fleet_models import AckReason, AckStatus, CommandResult, TerrainCode
from components.grid_rescue_mission import (
    AdjacentGridRescuePlanner,
    GridLayout,
    GridRescueMissionController,
    overlay_blocked_terrain,
)
from components.navigation import NavigationGoal, NavigationState
from components.radar_driver import RadarMount
from main import (
    CarMainApplication,
    MainConfig,
    build_argument_parser,
    configure_logging,
    default_log_dir,
    shutdown_logging,
)


# ==================== 比赛现场只需编辑以下三个列表 ====================
# 模拟取水地块：可临时改成 field 等，不要求一定是真实 lake/river。
WATER_PICKUP_TERRAINS = ["lake", "river"]
# 模拟灭火目标地块：通常保持 wildfire。
WILDFIRE_TARGET_TERRAINS = ["wildfire"]
# 禁止进入地块：当前无要求所以留空；如需避开居民地可填 ["settlements"]。
FORBIDDEN_TERRAINS = []
# =====================================================================

TERRAIN_NAME_TO_CODE = {
    "snow_mountain": int(TerrainCode.SNOW_MOUNTAIN),
    "field": int(TerrainCode.FIELD),
    "river": int(TerrainCode.RIVER),
    "settlements": int(TerrainCode.SETTLEMENTS),
    "lake": int(TerrainCode.LAKE),
    "debris_flow": int(TerrainCode.DEBRIS_FLOW),
    "wildfire": int(TerrainCode.WILDFIRE),
}

LOG = logging.getLogger("car-main")
NAVIGATION_STEP_TIMEOUT_S = 90.0


def terrain_codes(names):
    unknown = sorted(set(names) - set(TERRAIN_NAME_TO_CODE))
    if unknown:
        raise ValueError("unknown terrain names: {}".format(", ".join(unknown)))
    return tuple(TERRAIN_NAME_TO_CODE[name] for name in names)


class DisasterRescueApplication(CarMainApplication):
    def __init__(self, config: MainConfig) -> None:
        self._rescue_condition = threading.Condition()
        self._rescue_terminal_generation = 0
        self._rescue_terminal_state = None
        self._semantic_step = None
        self._semantic_lock = threading.Lock()
        self.rescue_controller = None
        super().__init__(config, hmac_key=None, fleet_bus=True)
        if self.fleet_node is None:
            raise RuntimeError("disaster rescue main requires FleetBus mode")
        # Intercept map installs at the task boundary. This also works with
        # older CarMainApplication versions that do not expose a map hook.
        self._base_navigation_set_map = self.navigation.set_map
        self.navigation.set_map = self._install_navigation_grid
        layout = GridLayout()
        planner = AdjacentGridRescuePlanner(
            water_terrain_codes=terrain_codes(WATER_PICKUP_TERRAINS),
            wildfire_terrain_codes=terrain_codes(WILDFIRE_TARGET_TERRAINS),
            forbidden_terrain_codes=terrain_codes(FORBIDDEN_TERRAINS),
            layout=layout,
        )
        self.rescue_controller = GridRescueMissionController(
            planner,
            navigate=self._navigate_rescue_cell,
            set_step_overlay=self._set_semantic_step,
            clear_overlay=self._clear_semantic_overlay,
            indicator=self._mission_indicator,
            on_result=self.fleet_node.set_active_command_result,
            hold_seconds=3.0,
        )
        self.fleet_node.set_disaster_handler(self.rescue_controller.submit)

    def _install_navigation_grid(self, grid) -> bool:
        with self._semantic_lock:
            step = self._semantic_step
        if step is not None:
            _current, _target, blocked = step
            grid = overlay_blocked_terrain(grid, blocked)
        return self._base_navigation_set_map(grid)

    def _set_semantic_step(self, current, target, blocked) -> None:
        with self._semantic_lock:
            self._semantic_step = (current, target, blocked)
        self._refresh_trusted_grid(force=True)
        LOG.info(
            "semantic rescue corridor current=%s target=%s blocked=%s",
            current,
            target,
            sorted(blocked),
        )

    def _clear_semantic_overlay(self) -> None:
        with self._semantic_lock:
            self._semantic_step = None
        self._refresh_trusted_grid(force=True)

    def _navigate_rescue_cell(self, cell) -> bool:
        if self.rescue_controller is None or self.rescue_controller.stop_requested:
            return False
        if cell is None:
            x_cm, y_cm = GridLayout().start_point_cm
            heading = 0.0
        else:
            x_cm, y_cm = GridLayout().centre(cell)
            heading = None
        with self._rescue_condition:
            generation = self._rescue_terminal_generation
        try:
            self._submit_console_goal(NavigationGoal(x_cm, y_cm, heading))
        except Exception as exc:
            LOG.error("rescue waypoint rejected cell=%s: %s", cell, exc)
            return False
        deadline = time.monotonic() + NAVIGATION_STEP_TIMEOUT_S
        with self._rescue_condition:
            while self._rescue_terminal_generation == generation:
                if self.rescue_controller.stop_requested:
                    return False
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    self._cancel_from_console()
                    LOG.error("rescue waypoint timeout cell=%s", cell)
                    return False
                self._rescue_condition.wait(min(0.2, remaining))
            return self._rescue_terminal_state is NavigationState.ARRIVED

    def _on_navigation_state(self, state: NavigationState, reason: str) -> None:
        super()._on_navigation_state(state, reason)
        if state not in (
            NavigationState.ARRIVED,
            NavigationState.FAILED,
            NavigationState.BLOCKED,
        ):
            return
        with self._rescue_condition:
            self._rescue_terminal_state = state
            self._rescue_terminal_generation += 1
            self._rescue_condition.notify_all()

    def _fleet_stop(self) -> CommandResult:
        if self.rescue_controller is not None:
            self.rescue_controller.stop()
        result = super()._fleet_stop()
        with self._rescue_condition:
            self._rescue_condition.notify_all()
        return result

    @staticmethod
    def _mission_indicator(stage: str, active: bool) -> None:
        # 保留统一声光外设接入点；当前硬件仓库尚未提供小车灯/蜂鸣器引脚驱动。
        # 终端响铃和日志可用于台架确认，接线确定后只需替换此函数。
        if active:
            print("\a{} indicator ON".format(stage), flush=True)
            LOG.warning("rescue indicator ON stage=%s", stage)
        else:
            LOG.info("rescue indicator OFF stage=%s", stage)

    def close(self) -> None:
        if self.rescue_controller is not None:
            self.rescue_controller.stop()
        super().close()


def main(argv=None) -> int:
    parser = build_argument_parser()
    parser.description = __doc__
    parser.set_defaults(fleet_bus=True)
    args = parser.parse_args(argv)
    requested_log_dir = default_log_dir() if args.log_dir is None else Path(args.log_dir)
    try:
        configure_logging(requested_log_dir, args.log_level)
    except OSError as exc:
        print("cannot create detailed log in {}: {}".format(requested_log_dir, exc), file=sys.stderr)
        return 2
    app = None
    try:
        config = MainConfig(
            radar_port=args.radar_port,
            link_port=args.link_port,
            radar_mount=RadarMount(
                args.radar_x_cm,
                args.radar_y_cm,
                args.radar_yaw_cw_deg,
            ),
            startup_scan_count=args.startup_scans,
            calibration_timeout_s=args.calibration_timeout,
            allow_reverse=args.allow_reverse,
            console_enabled=not args.no_console,
        )
        app = DisasterRescueApplication(config)

        def stop_handler(signum, frame) -> None:
            LOG.info("received signal %s; stopping disaster rescue main", signum)
            app.request_stop()

        signal.signal(signal.SIGINT, stop_handler)
        signal.signal(signal.SIGTERM, stop_handler)
        app.run()
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception:
        LOG.exception("disaster rescue main failed")
        return 1
    finally:
        try:
            if app is not None:
                app.close()
        finally:
            shutdown_logging()


if __name__ == "__main__":
    raise SystemExit(main())
