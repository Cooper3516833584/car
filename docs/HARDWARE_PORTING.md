# 硬件移植指南（HARDWARE_PORTING.md）

换主控板（例如 ROCK 5A → 树莓派 / Jetson / RK3566 等）**不需要修改**：

- `code/main_task1.py` / `code/main_task2.py`（任务入口）
- `code/main_radar_camera_line_following.py`（比赛流程）
- `code/components/navigation.py`（Hybrid A* / Pure Pursuit / 坐标约定）
- `code/components/radar_driver.py`（D500 协议 / ICP / 墙线融合）
- 所有比赛业务逻辑

只需要改**一份 TOML 配置**，以及（仅当新板子的 PWM/GPIO API 与 Linux sysfs
不兼容时）在 `code/hal/` 里新增一个 backend。

## 1. 需要处理的项目

### PWM（舵机）

配置节：`[hardware.steering_pwm]`

```toml
backend = "linux-sysfs"        # 当前实现
channel = 0
period_ns = 20000000           # 50 Hz
polarity = "normal"
chip_device_match = "fd8b0000.pwm"   # 按新板子实际设备树节点名改
```

如果新板子使用 **libgpiod**、**pigpio** 或厂商 PWM API：
在 `code/hal/pwm.py` 增加一个新的 `PWMOutput` 实现（例如
`LibgpiodPWMOutput`），在 `code/config/factory.py` 的 `build_steering_servo`
里按 `backend` 选择即可。**不要**在主程序里写 `if board == "rock5a": ...`。

### GPIO（声光报警）

配置节：`[hardware.alarm_gpio]`

```toml
backend = "linux-sysfs-bank"
sysfs_root = "/sys/class/gpio"
bank_label = "gpio4"     # 新板子的 gpiochip label
line_offset = 11         # 相对 chip base 的偏移
active_low = true        # 低电平触发=报警开
```

新 API 同样在 `code/hal/gpio.py` 增加 `DigitalOutput` 实现。

### 串口设备

```toml
[devices.motor]   port = "/dev/ttyACM0"   # C10B 驱动板
[devices.radar]   port = "/dev/ttyS6"     # D500 雷达 UART
[devices.hc14]    port = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
[devices.screen]  port = "..." ; baudrate = 9600
```

### 摄像头设备

```toml
[devices.camera]
source = 0        # 或 "/dev/video2"
width = 640
height = 360
fps = 30.0
fourcc = "MJPG"
backend = "v4l2"
```

## 2. Linux 用户权限

板端运行需要以下权限，否则串口/GPIO/PWM 打开失败：

| 资源 | 需要的权限 |
|---|---|
| `/dev/ttyACM0`、`/dev/ttyS6`、`/dev/ttyUSB0`、串口屏 | `dialout` 组 |
| `/sys/class/gpio` | root 或 `gpio` 组（配合 `sound-light-alarm.service` 的 chown） |
| `/sys/class/pwm` | root（或 `pwm` 组 + udev 规则） |

```bash
sudo usermod -aG dialout,gpio,pwm your-user
```

运行比赛程序通常使用 `sudo -E` 或一个带这些组的 systemd service。

## 3. Device Tree / Pinmux

- 舵机 PWM：新板子需要启用对应 PWM 通道的设备树 overlay（ROCK 5A 用
  `rk3588-pwm0-m2`），并在 `[hardware.steering_pwm]` 填 `chip_device_match`。
- D500 UART：新板子需要启用对应 UART 的 overlay（ROCK 5A 用
  `rk3588-uart6-m1`），并确认出现对应的 `/dev/ttySx`，填入
  `[devices.radar] port`。

具体接线信息属于**当前车辆文档**（见 `docs/platforms/rock5a.md`），
不会埋在业务代码的报错信息里。

## 4. 验证步骤（移植后）

1. `python3 -m compileall code` 通过；
2. `pytest code/test` 全部通过（单元测试不访问真实硬件）；
3. 用 `--fleet-position-only` 先只开雷达/建图，确认定位正常；
4. 低速直线测试 → 低速转弯 → 再按 `docs/CALIBRATION.md` 完整标定。
