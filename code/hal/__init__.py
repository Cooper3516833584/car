"""Hardware abstraction layer for the competition car.

Board-specific backends (Linux sysfs PWM/GPIO today) are selected by the TOML
profile.  Adding a new board with a different API only requires a new HAL
backend here; the competition main program never branches on board names.
"""

from .pwm import (
    LinuxSysfsPWMOutput,
    PWMBackendError,
    PWMOutput,
)
from .gpio import (
    DigitalOutput,
    GPIOBackendError,
    LinuxSysfsBankGPIOOutput,
    resolve_gpio_number,
)

__all__ = [
    "LinuxSysfsPWMOutput",
    "PWMBackendError",
    "PWMOutput",
    "DigitalOutput",
    "GPIOBackendError",
    "LinuxSysfsBankGPIOOutput",
    "resolve_gpio_number",
]
