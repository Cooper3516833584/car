# 配置说明（CONFIGURATION.md）

本仓库的比赛程序已经从“代码里写死当前车辆参数”重构为 **TOML 配置驱动**。
同一份主程序可以原样运行在不同学校的小车上，只需要换一份配置文件。

```bash
# 用指定配置运行（推荐）
python3 code/main_task1.py --config configs/cooper_rock5a_l150.toml
python3 code/main_task2.py --config configs/cooper_rock5a_l150.toml

# 不传 --config 时按优先级自动解析到当前实车配置
python3 code/main_task1.py
```

## 1. 配置路径优先级

```
CLI  --config 参数
  >  环境变量 CAR_CONFIG
  >  仓库默认 configs/cooper_rock5a_l150.toml
```

如果最终解析到的文件不存在，程序会**明确报错并退出**，绝不会默默用一组隐藏的
硬编码参数继续驱动车辆。

## 2. 配置结构

| Section | 用途 | 换车时通常要改？ |
|---|---|---|
| `[profile]` | 配置名/描述 | 建议改 |
| `[hardware.steering_pwm]` | 舵机 PWM（sysfs 芯片匹配、周期、极性） | 换主控板必改 |
| `[hardware.alarm_gpio]` | 声光报警 GPIO（bank、line、active_low） | 换主控板必改 |
| `[devices.motor]` | C10B 驱动板串口设备 | 必改 |
| `[devices.radar]` | D500 雷达串口设备 | 必改 |
| `[devices.hc14]` | HC-14 无线串口设备 | 按实际 USB 口 |
| `[devices.screen]` | 串口屏设备/波特率 | 按实际设备 |
| `[devices.camera]` | 摄像头 source/分辨率/帧率/FOURCC | 按实际摄像头 |
| `[vehicle.geometry]` | 实测车身几何（轴距/轮距/车身/后轴偏移） | 必改（实测） |
| `[vehicle.drive]` | C10B 固件轮距、最小半径、速度限幅 | 换固件才改 |
| `[vehicle.steering]` | 舵机标定（方向/范围/中位/厂家曲线） | 必改（实测标定） |
| `[sensors.radar.mount]` | 雷达相对后轴中心的安装位置/偏航 | 必改（实测） |
| `[sensors.camera.perspective]` | 摄像头透视标定四点/地面尺寸 | 必改（重新标定） |
| `[sensors.camera.line]` | 黑线视觉参数 | 按赛道视觉重新标定 |
| `[missions.common]` | 起步距离/建图圈数/超时/开关 | 按需 |
| `[missions.task1]` / `[missions.task2]` | 每段速度、任务请求状态、完成报警时长 | 按规则调 |
| `[missions.control]` | 比赛控制调参（相机纠偏增益、AB/BC/终段参数） | 视觉重标定后调 |
| `[runtime]` | 现场可改状态（radar center 选择文件） | 一般不改 |
| `[schema]` | 配置版本 | 不改 |

## 3. 两套“轮距”的区别（重要）

```toml
[vehicle.geometry]
physical_track_width_mm = 117.1   # 真实左右后轮中心距：阿克曼几何/转弯半径/导航规划

[vehicle.drive]
firmware_track_width_mm = 164.0   # C10B 固件编译值：Vz=(right-left)/firmware_track
```

- `physical_track_width_mm`：真实测量值，用于**车身物理几何**。
- `firmware_track_width_mm`：C10B 固件内部反算左右后轮用的**协议常数**，
  不一定等于实体轮距。

**禁止把 `117.1` 传给 C10B 协议逆变换**：否则固件产生的后轮差速会放大
`164/117.1 ≈ 1.40` 倍，和前轮舵角不匹配。仓库测试
`test_vehicle_config.py` / `test_ackermann_drive.py` 专门防止两者被合并。

## 4. CLI 覆盖

所有正式 CLI 参数都保留，且遵循 **CLI 显式值 > 配置文件值**。argparse 默认值
是 `None`，只有用户显式传入时才覆盖 TOML：

```bash
python3 code/main_task2.py \
  --radar-port /dev/ttyS6 \
  --radar-center-behind-a-cm 36.5 \
  --ab-speed-cm-s 18 \
  --no-camera-correction \
  --fleet-link-port /dev/ttyUSB0
```

常用参数：`--radar-port`、`--radar-x-cm`、`--radar-y-cm`、`--radar-yaw-cw-deg`、
`--startup-scans`、`--calibration-timeout`、`--radar-center-behind-a-cm`、
`--ab-speed-cm-s`、`--bc-speed-cm-s`、`--cd-speed-cm-s`、`--cd-second-speed-cm-s`、
`--da-speed-cm-s`、`--camera`、`--no-camera-correction`、`--fleet-link-port`、
`--no-fleet-position`、`--fleet-position-only`、`--wait-for-fleet-start`、
`--fleet-mission-request-state`、`--completion-alarm-seconds`、`--config`。

## 5. Runtime state（现场可改状态）

比赛现场允许在 `20 cm` 和 `36.5 cm` 之间切换“雷达中心在 A 点后方距离”。
该选择**不属于车辆静态标定**，保存在：

```
runtime/car_state.json
```

（路径和允许值由 `[runtime] state_file` / `allowed_radar_center_behind_a_cm`
配置。）程序不会改写整个 TOML。

加载优先级：

```
CLI 显式 --radar-center-behind-a-cm
  >  runtime/car_state.json
  >  [missions.common] radar_center_behind_a_cm
```

## 6. 舵角限幅的派生

主程序融合相机纠偏后会对最终舵角做限幅。默认值**从车辆档案自动派生**，无需手写：

- 最小值 = `vehicle.steering.logical_right_max_rad`（舵机机械右极限）
- 最大值 = 满足 `vehicle.drive.min_turn_radius_mm` 的几何左极限

当前实车派生为 `-0.32 / +0.336 rad`。如确需手工覆盖，可在
`[missions.control]` 写 `steering_min_rad` / `steering_max_rad`。

## 7. 协议/算法常量不配置化

D500 47 字节帧、`54 2C` 帧头、CRC-8/0x4D、C10B 11 字节速度帧、XOR 校验、
`BB 33`/`AA 22`、FleetBus 帧、坐标系正负方向、Hybrid A*/Pure Pursuit/ICP、
比赛固定赛道几何等，都属于协议或算法定义，**保留在代码中**，不会出现在 TOML。
