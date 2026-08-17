"""Unified TOML configuration for the competition car.

This package is the *composition/root* layer: it loads the TOML profile,
validates it into strongly-typed dataclasses and builds validated application
objects.  Business components never load TOML themselves; they receive already
validated objects built here.
"""

from .models import (
    AlarmGPIOConfig,
    CameraDeviceConfig,
    CameraLineConfig,
    CameraPerspectiveConfig,
    CarConfig,
    DevicesConfig,
    HC14DeviceConfig,
    HardwareConfig,
    MissionCommonConfig,
    MissionControlConfig,
    MissionsConfig,
    MotorDeviceConfig,
    ProfileConfig,
    RadarDeviceConfig,
    RadarMountConfig,
    RuntimeStateConfig,
    ScreenDeviceConfig,
    SensorCameraConfig,
    SensorRadarConfig,
    SensorsConfig,
    SteeringCalibrationConfig,
    SteeringPWMConfig,
    Task1Config,
    Task2Config,
    VehicleConfig,
    VehicleDriveConfig,
    VehicleGeometryConfig,
)
from .loader import (
    CAR_CONFIG_ENV_VAR,
    DEFAULT_CONFIG_FILENAME,
    ConfigError,
    load_car_config,
    resolve_config_path,
)
from .runtime_state import (
    RuntimeRadarCenterState,
    load_runtime_radar_center_cm,
    save_runtime_radar_center_cm,
)
from .factory import (
    build_steering_calibration,
)

__all__ = [
    "AlarmGPIOConfig",
    "CameraDeviceConfig",
    "CameraLineConfig",
    "CameraPerspectiveConfig",
    "CarConfig",
    "DevicesConfig",
    "HC14DeviceConfig",
    "HardwareConfig",
    "MissionCommonConfig",
    "MissionControlConfig",
    "MissionsConfig",
    "MotorDeviceConfig",
    "ProfileConfig",
    "RadarDeviceConfig",
    "RadarMountConfig",
    "RuntimeStateConfig",
    "ScreenDeviceConfig",
    "SensorCameraConfig",
    "SensorRadarConfig",
    "SensorsConfig",
    "SteeringCalibrationConfig",
    "SteeringPWMConfig",
    "Task1Config",
    "Task2Config",
    "VehicleConfig",
    "VehicleDriveConfig",
    "VehicleGeometryConfig",
    "CAR_CONFIG_ENV_VAR",
    "DEFAULT_CONFIG_FILENAME",
    "ConfigError",
    "load_car_config",
    "resolve_config_path",
    "RuntimeRadarCenterState",
    "load_runtime_radar_center_cm",
    "save_runtime_radar_center_cm",
    "build_steering_calibration",
]
