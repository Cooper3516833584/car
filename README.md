# 小车上位机代码结构

正式入口为 `code/main.py`，可复用硬件模块放在 `code/components`，所有测试、临时脚本和一次性配置工具放在 `code/test`。

## 正式驱动组件

- `components/rear_motor.py`：C10B 后轮串口控制、20 Hz 刷新、超时停车。
- `components/steering_servo.py`：ROCK 5A Pin 23 前轮转向舵机及厂家标定曲线。
- `components/ackermann_drive.py`：统一设置车速、前轮偏航方向，并可联动后轮差速。

`main.py` 通过 `Navigation` 间接使用 `AckermannDrive`；只有维护、标定或特殊控制时才直接访问前后独立组件。

## 后驱电机组件

文件：`components/rear_motor.py`

硬件链路为 ROCK 5A 的 `/dev/ttyACM0` -> WHEELTEC C10B 驱动板，串口参数 `115200 8N1`。组件生成已经实车验证的 11 字节帧：

```text
7B 00 00 VX_H VX_L 00 00 VZ_H VZ_L BCC 7D
```

`Vx` 的单位是 `mm/s`，`Vz` 的单位是 `mrad/s`，BCC 为前 9 字节逐字节异或。

### 控制接口

- 两轮联动：`set_linked(speed_mm_s, MotorDirection.FORWARD/REVERSE)`。
- 两轮原子化分开设置：`set_wheels(left_mm_s, right_mm_s)`；正数前进，负数后退。
- 单独更新一侧：`set_left(speed_mm_s)`、`set_right(speed_mm_s)`，另一侧保持最近目标。
- 立即停车：`stop()`；释放资源：`close()`。
- 推荐用 `with RearMotorDriver(...) as motors:`，保证异常退出时仍发送 5 个停止帧。

调用方必须以快于 `command_timeout_s` 的频率刷新目标；默认超时 `0.5 s` 后组件自动发送零速。组件默认以 `20 Hz` 刷新命令，默认速度限幅为 `+/-300 mm/s`，将来低速实车标定完成后才能按需提高，固件线速度绝对上限为 `1200 mm/s`。

示意代码（不是 `main`）：

```python
from components.rear_motor import MotorDirection, RearMotorDriver

with RearMotorDriver() as motors:
    motors.set_linked(100, MotorDirection.FORWARD)  # 左右轮同步前进
    # 应用循环必须在 0.5 秒内再次刷新命令
    motors.set_wheels(80, 120)                      # 同向、不同速度
    motors.stop()
```

### “独立控制”的固件边界

当前阿克曼固件没有公开左右电机原始命令，而是先接收车体命令，再计算：

```text
left  = Vx - Vz * 0.082
right = Vx + Vz * 0.082
```

所以组件用以下逆变换实现左右轮目标：

```text
Vx = (left + right) / 2
Vz = (right - left) / 0.164
```

固件强制最小转弯半径 `350 mm`，因此两轮同向但速度略有差别可以准确控制；单轮转动、左右反转和原地旋转无法由当前固件实现，组件会抛出 `UnsupportedWheelCommand`。如果后续确实要求这些动作，必须另行设计并刷写 C10B 的“左右轮直接目标”扩展协议，不能只修改 ROCK 5A 上位机。

## 前轮转向舵机

舵机信号位于 ROCK 5A 物理 `Pin 23`（`PWM0_M2`），PWM 频率为 `50 Hz`。设备树覆盖项 `rk3588-pwm0-m2` 已启用，组件通过 `/sys/class/pwm` 自动找到 `fd8b0000.pwm`。访问 sysfs PWM 通常需要 root 权限。

采用 WHEELTEC 源码中的三次标定，不使用线性猜测。本车实测舵机/连杆安装方向与厂家
曲线的车辆方向相反，因此先把逻辑车辆转角取反后再代入厂家曲线：

```text
calibration_theta = -vehicle_theta
factory_PWM_us = 1500 + 640.62 * (-0.628*calibration_theta^3
                                  + 1.269*calibration_theta^2
                                  - 1.772*calibration_theta + 0.001)
PWM_us = clamp(
    factory_PWM_us + (1580 - 1501),
    800,
    2200
)
```

约定 `vehicle_theta > 0` 为车辆左偏航、`vehicle_theta < 0` 为车辆右偏航。取反后的
逻辑机械范围不对称：右侧最多 `-0.32 rad`，左侧最多 `+0.49 rad`；运动时还要服从
`350 mm` 最小转弯半径，因此 Navigation 当前实际限制约为右 `-0.32 rad`、左
`+0.336 rad`。

| 偏航 | 转角 | PWM脉宽 |
|---|---:|---:|
| 右最大 | `-0.32 rad` | `1286 us` |
| 右 | `-0.20 rad` | `1382 us` |
| 右 | `-0.12 rad` | `1454 us` |
| 回中（本车实测） | `0` | `1580 us` |
| 左 | `+0.12 rad` | `1728 us` |
| 左 | `+0.20 rad` | `1842 us` |
| 左（Navigation 半径边界附近） | `+0.32 rad` | `2039 us` |
| 左机械最大 | `+0.49 rad` | `2200 us` |

独立接口：

```python
from components import FrontSteeringServo, YawDirection

with FrontSteeringServo() as steering:
    steering.set_yaw(YawDirection.LEFT, 0.12)
    steering.set_angle(-0.20)  # 负值：右偏航
    steering.center()
```

组件启动时先回中；正常关闭或异常退出时也回中，并保持 PWM 使能以维持前轮中位。维护时才调用 `disable()` 释放舵机保持力。

## 前轮与后轮联动

统一组件用法：

```python
from components import AckermannDrive, MotorDirection, YawDirection

with AckermannDrive() as drive:
    # 100 mm/s 前进，左偏 0.12 rad，后轮按阿克曼几何自动差速。
    drive.set_motion(100, 0.12)

    # 保持当前速度，改为右偏 0.20 rad，继续联动后轮。
    drive.set_yaw(YawDirection.RIGHT, 0.20)

    # 也可以直接使用有符号转角：正数左偏、负数右偏。
    drive.set_steering(-0.12)

    # 保持当前转角，仅把速度改成 80 mm/s 反向。
    drive.set_speed(80, MotorDirection.REVERSE)

    drive.stop(center_steering=True)
```

联动必须区分实体几何和驱动板协议几何。实体小车使用实测轮距 `117.1 mm`、
轴距 `142.5 mm`，先根据前轮转角计算有符号转弯半径和所需后轮差速：

```text
R = 142.5 / tan(theta) - 117.1 / 2
omega = vehicle_speed / R
left  = vehicle_speed - omega * 117.1 / 2
right = vehicle_speed + omega * 117.1 / 2
```

C10B 固件内部仍按编译值 `164 mm` 把 `Vx/Vz` 还原成左右轮速度，因此串口命令
必须再用协议轮距反算：

```text
Vx = (left + right) / 2
Vz = (right - left) / 164
```

这两套轮距不能合并。若错误地用 `117.1 mm` 计算 `Vz`，驱动板实际产生的后轮差速
会放大约 `164/117.1 = 1.40` 倍，与前轮舵角不匹配。

默认 `rear_differential_linked=True`：左转时左后轮较慢、右后轮较快，右转相反；倒车会自动反转相应偏航角速度。若某个低速测试只想转舵机而不做后轮差速，可传 `rear_differential_linked=False`，但车辆运动时不建议关闭，否则会增加轮胎侧滑。

所有角度、转弯半径和内外轮速度会先完整校验，再写入 PWM 和串口。高速大转角导致外侧轮超过默认 `300 mm/s` 限幅时会拒绝整条命令；调用方应降低中心速度，不会由组件静默缩放。

后轮的 `0.5 s` 命令看门狗继续有效，所以运动状态下 `main` 需要周期性调用 `set_motion()`、`set_speed()` 或 `set_yaw()`。仅改变一次舵机并不能永久维持后轮运动命令。

## HC-14 串口通信组件

文件：`components/serial_communication.py`。该组件只负责串口传输，不解释地面站业务，不保存 HMAC 密钥，也不会调用电机或舵机：

- 小车稳定串口路径：`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`。
- `115200 8N1`，无软硬件流控。
- 打开串口时明确清除 `DTR` 和 `RTS`，保证 HC-14 处于透明传输而不是 AT 模式。
- 后台读取、断线检测和按 `1 s` 间隔自动重连。
- 写入断线时明确报错，不缓存过期业务帧。
- 不依赖 `pyserial`，使用 ROCK 5A 自带的 Linux `termios`、`select` 和 `fcntl`。

当前地面站串口层会在内部 GroundStationLink 帧外增加：

```text
BB 33 | bridge_len:u8 | AA 22 ... GroundStationLink frame
```

因此组件默认 `bridge_envelope=True`。调用方传给 `write()` 的是一个完整内部 `AA 22...` 帧；`on_bytes` 每次收到的也是一个已经去掉 `BB 33` 外层的完整内部帧。协议HMAC、消息类型、ACK、重发和重复包过滤应由以后单独的业务协议组件处理。

调用示例（不是 `main`）：

```python
from components import SerialCommunicationDriver

def on_ground_frame(frame: bytes) -> None:
    # frame 是完整 AA 22... 帧；这里只转交业务解析器。
    protocol_component.feed(frame)

link = SerialCommunicationDriver(
    on_bytes=on_ground_frame,
    on_connected=lambda: print("ground link connected"),
    on_disconnected=lambda error: print("ground link disconnected", error),
)
link.start()
if link.wait_connected(2.0):
    link.write(protocol_component.build_outbound_frame())
link.close()
```

`start()` 为异步连接，调用方可使用 `connected`、`wait_connected(timeout)` 或连接回调判断是否可写。推荐最终程序用上下文管理器或在 `finally` 中调用 `close()`。

## 资料依据

- WHEELTEC 附送源码：`5.STM32源码/LD14雷达/L150-避障巡线雷达小车-C10B-HAL库-20260525.zip`。
- 固件文件 `CONTROL/bluetooth.c`：USB 11 字节解析、`Vz_to_Akm_Angle()` 和 `350 mm` 最小转弯半径。
- 固件文件 `CONTROL/control.c`、`CONTROL/control.h`：阿克曼后轮逆运动学、轮距 `0.164 m`、轴距 `0.144 m` 和速度限幅。
- 固件 `CONTROL/control.c` 中的舵机标定多项式和机械转角范围。
- 本车实测：`100 mm/s` 直行帧可驱动两只后轮；机械中位实测为 `1580 us`。
  2026-07-22 实车日志与前轮目测确认厂家车辆方向在本车上必须取反；逻辑转向
  `-0.12/0/+0.12 rad` 分别对应 `1454/1580/1728 us`。后轮偏航符号和雷达航向
  变化一致，禁止再反转 Navigation 或后轮差速来补偿前轮。
- 地面站活动代码：`Ground_Station/components/serial_transport.py`，用于 `BB 33` 桥封装、`115200 8N1`、DTR/RTS状态、后台收发和重连行为；小车组件保持字节级兼容，但改用标准库以避免ROCK 5A缺少 `pyserial`。
- 地面站 `details.md`：当前 GroundStationLink V2 串口分层和真实 HC-14 参数/双向验证记录。

## D500 雷达、定位与无人机全局坐标

正式组件为 `components/radar_driver.py`，默认只读 ROCK 5A 的
`/dev/ttyS6`（UART6_M1，`230400 8N1`）。它包含以下相互独立的层：

- `D500PacketParser`：增量解析 `54 2C`、47 字节、12 点数据帧，校验
  CRC-8/0x4D，遇到噪声或坏帧后自动重新同步；
- `RadarScanAssembler`：按角度过零拼成完整一圈，启动后的残缺首圈直接丢弃；
- `RadarOdometry` / `ICPScanMatcher`：参考无人机雷达组件，以 SVD-ICP
  做相邻点云定位，并增加对应距离、单步位移、航向和误差门限；
- `WallLineLocalizer` / `fuse_wall_observation`：周期性识别矩形场地的后墙和右墙，
  产生绝对墙距/航向观测，以有限增益修正 ICP 累计漂移；
- `DroneGlobalAlignment`：用同一物理位置在“小车局部地图”和“无人机全局地图”
  中的两个参考位姿，求出固定旋转和平移；
- `DroneGlobalPointMap`：只接收变换完成的无人机全局点，默认 5 cm 栅格。

坐标约定与无人机工程完全一致：`+X` 向前、`+Y` 向左，航向角单位为度，
**顺时针（向右转）为正**；位姿和地图使用厘米。D500 原始角度也是从雷达前方
零度开始、顺时针增加，所以原始极坐标转换为 XY 时采用
`x=d*cos(a), y=-d*sin(a)`。这与车辆控制组件中“转向角正数表示左转”的约定
方向相反：若把车辆转向推算的航向增量交给雷达定位，必须先取反，不能直接混用。

雷达安装方向单独通过 `RadarMount` 描述。默认值只适用于“雷达零度方向与车头
一致、雷达位于车辆坐标原点”；若实车安装有旋转或偏移，必须填写实际测量值。
全局方向差不写死，使用参考位姿标定：

```python
from components import D500RadarComponent, Pose2D, RadarMount

def on_radar(update):
    if update.global_pose is None:
        return  # 尚未完成全局参考标定，不产生全局地图
    send_pose_and_points(update.global_pose, update.global_points_cm)

radar = D500RadarComponent(
    mount=RadarMount(
        x_forward_cm=8.0,  # 示例值，必须替换成实测安装尺寸
        y_left_cm=0.0,
        yaw_cw_deg=0.0,
    ),
    on_update=on_radar,
)

# 两个位姿必须表示标定瞬间同一个物理车体位姿。
radar.set_global_reference(
    car_local_pose=Pose2D(0.0, 0.0, 0.0),
    drone_global_pose=Pose2D(420.0, -135.0, 90.0),
)
radar.start()
```

### ICP 与矩形墙线融合

融合默认关闭，必须先完成雷达全局参考标定，并明确场地“后墙与右墙交点”以及场地
`+X` 方向在无人机全局坐标中的位姿。墙体局部坐标约定为：后墙 `x=0`、右墙
`y=0`、`+X` 从后墙指向场内、`+Y` 从右墙指向场内。

```python
from components import (
    DroneGlobalAlignment,
    Pose2D,
    RectangularWallReference,
    WallFusionConfig,
)

# 示例：后墙/右墙交点在无人机全局 (300, -200) cm，场地 +X 相对
# 无人机全局 +X 顺时针旋转 90°。数值必须由现场测量替换。
wall_to_global = DroneGlobalAlignment.from_reference(
    car_local_pose=Pose2D(0, 0, 0),
    drone_global_pose=Pose2D(300, -200, 90),
)
radar.enable_wall_fusion(
    RectangularWallReference(wall_to_global),
    fusion_config=WallFusionConfig(
        update_every_scans=1,  # 每个完整圆周都尝试一次
        position_gain=0.20,    # 每次只修正 20% 位置残差
        yaw_gain=0.15,         # 每次只修正 15% 航向残差
        consistency_samples=3, # 连续三次绝对观测一致后才允许回写
    ),
)
```

墙线修正不会因为增益计算结果超过单圈限幅而整次丢弃。绝对观测连续一致后，
位置每圈最多修正 `2 cm`、航向每圈最多修正 `0.5°`。墙线与 ICP 的位置残差达到
`15 cm` 时 Navigation 进入 `RELOCALIZING` 并保持停车；残差连续两次回到 `5 cm`
以内后，保留原目标并从修正后的当前位置重新规划。

每个完整圆周仍先由 ICP 推进连续位姿。到达配置周期后，墙线定位器使用 ICP 预测把
点云转换到墙体坐标系，对后墙和右墙分别做候选距离筛选、离群点剔除和 PCA 直线拟合；
直线必须满足最少点数、长度、RMS、轴向角和两墙正交一致性门限。通过质量检查后，
墙距/航向还必须通过相对 ICP 预测的最大位置和航向残差门限，才按有限增益回写
`RadarOdometry.pose`。纠正后的同一位姿用于全局点云建图和 Navigation，避免定位与
地图使用两套姿态。

若墙被遮挡、线段过短、残差过大或只看到其中一面墙，组件不会中断 ICP：有效的单墙
可以只纠正对应坐标和航向；没有有效墙线时继续使用 ICP。该方法用于纠正缓慢累计
漂移，不负责从严重错误的初始位置重新定位；默认候选关联范围为 `45 cm`。
`RadarLocalizationUpdate.wall_fusion` 可查看本次是否尝试、是否接受、观测点数、
拟合 RMS 和拒绝原因。

没有调用 `set_global_reference()` 或没有在构造器传入已标定的
`DroneGlobalAlignment` 时，组件仍可解析雷达并输出小车局部里程计，但刻意不写入
全局地图，防止用一个猜测的朝向污染无人机地图。ICP 依赖 `numpy`；串口、协议、
坐标变换和栅格本身只用 Python 标准库。

ROCK 5A 使用前需要在设备树覆盖中启用 `UART6-M1`，并确认出现 `/dev/ttyS6`。
该链路只有 D500 TX 接到 ROCK RX，驱动以只读方式打开 UART，不会控制雷达 PWM
或小车驱动板。

本节本地资料依据：

- 无人机雷达采集与定位：
  `C:\Users\TZDEZACR\Desktop\DDDDDrone_Cloned\python_sdk\FlightController\Components\LDRadar_Driver.py`、
  `LDRadar_Resolver.py`、`Utils.py`；
- 无人机点云匹配：
  `C:\Users\TZDEZACR\Desktop\DDDDDrone_Cloned\python_sdk\FlightController\Solutions\Radar_SLAM.py`；
- 无人机全局坐标旋转定义：
  `C:\Users\TZDEZACR\Desktop\DDDDDrone_Cloned\python_sdk\warehouse_radar_localizer.py`。
- 学长矩形墙线定位入口：
  `C:\Users\TZDEZACR\Desktop\DDDDDrone_Cloned\python_sdk\former_code\2026_radar.py`，
  以及其实际调用的 `FlightController\Solutions\Radar_SLAM.py::radar_resolve_rt_pose`。

## Navigation 自主导航组件

正式组件为 `components/navigation.py`。组件使用 Hybrid A* 在占据栅格上规划满足
阿克曼最小转弯半径的路径，再由“Pure Pursuit 曲率前馈 + 雷达位姿反馈”联动已有
`AckermannDrive`。控制线程至少以 `20 Hz` 刷新驱动看门狗；每收到一个通过门限的
D500 完整圆周定位结果，`update_from_radar()` 会立即唤醒控制线程，不必等到下一个
固定周期。支持仅到达目标位置，也支持到达目标附近时满足指定车头朝向。

带最终朝向的前进任务会先尝试使用保守对称转弯半径生成 Dubins 解析连接，并对整条
曲线按 `2.5 cm` 间隔执行旋转车身矩形碰撞检查；解析曲线被障碍挡住时才回退 Hybrid
A*。Hybrid A* 单次搜索默认最多 `5 s`，并会响应停止、取消、暂停和退出，避免复杂
朝向任务长期占满 CPU。超时会停车并报告 `planning timed out after 5.0s`，不会在后台
继续算完后突然发车。

行驶中的闭环不再只是下发固定速度：控制器把当前雷达位姿投影到剩余路径段，计算
有符号横向偏差和车辆航向偏差，再与 Pure Pursuit 前视曲率叠加生成前轮转角。车辆
位于路径左侧时会向右修正，位于右侧时会向左修正；倒车时根据阿克曼运动学自动反转
反馈符号。路径进度只允许向前推进，避免交叉路径或墙线小幅纠漂使跟踪点跳回旧路段。

默认反馈与安全参数位于 `PurePursuitConfig` / `NavigationConfig`：

- 横向/航向反馈增益分别为 `0.35`、`0.65`，偏差增大时同步降低车速；
- ICP 平均残差从 `4 cm` 开始降速，到 `10 cm` 时降至正常跟踪速度的 `40%`；
- 前轮转角变化率限制为 `1.2 rad/s`，防止单圈点云抖动造成舵机猛跳；
- Pure Pursuit 曲率叠加横向/航向反馈后再次按左右实体最小转弯半径限幅；当前
  右转受机械范围限制为 `-0.32 rad`（半径约 `489 mm`），左转受最小半径限制为
  约 `+0.336 rad`（半径 `350 mm`）；
- 超过 `35 cm` 的路径偏差须由 `3` 个不同雷达位姿连续确认才重规划，超过
  `60 cm` 则立即停车重规划；拒绝的 ICP 圈不会刷新定位时间，连续 `0.5 s` 没有
  可用定位就停车回中。

这些默认值是保守初值，实车调参应先保持低速。横向摆动时优先降低
`cross_track_gain` 或 `max_steering_rate_rad_s`；回线太慢时小幅提高
`cross_track_gain`；车头左右摆动时降低 `heading_gain`。不要先提高车速掩盖定位或
安装外参问题。

实车默认几何尺寸：

| 参数 | 数值 |
|---|---:|
| 车轮厚度 | `26.4 mm` |
| 左右车轮外侧总宽 | `143.5 mm` |
| 左右轮中心距 | `143.5 - 26.4 = 117.1 mm` |
| 前后轴距 | `142.5 mm` |
| 矩形车身 | `230 × 145 mm` |
| 驱动固件最小转弯半径 | `350 mm` |

Navigation 位姿原点定义为**后轴中心**。在车身前后余量对称的假设下，碰撞矩形
中心位于后轴前方 `71.25 mm`；若后续测得前后悬长度不对称，应修改
`VehicleGeometry.rear_axle_to_body_center_cm`。雷达的 `RadarMount` 也必须以这个
后轴中心为车体原点填写雷达安装位置，否则导航位姿和车身碰撞范围会出现固定偏差。

导航地图的坐标定义为：`+X` 是无人机 `0°`，`+Y` 是俯视时 `+X` 左侧；航向角
俯视逆时针为正并归一化为 `0～359°`。雷达位姿使用顺时针正角，因此组件入口
`update_from_radar()` 固定执行：

```text
navigation_heading_deg = (-radar_yaw_cw_deg) % 360
```

典型调用方式（不是 `main.py`）：

```python
from components import (
    Navigation,
    NavigationConfig,
    NavigationGoal,
    OccupancyGrid,
)

navigation = Navigation(
    config=NavigationConfig(
        allow_reverse=False,  # 改为 True 才允许规划倒车
    )
)
navigation.start()

navigation.set_map(occupancy_grid)
radar.on_update = navigation.update_from_radar

# 只要求抵达坐标附近。
navigation.set_goal(NavigationGoal(x_cm=500, y_cm=240))
navigation.start_navigation()

# 或要求抵达后车头约为逆时针 90°。
navigation.set_goal(
    NavigationGoal(
        x_cm=500,
        y_cm=240,
        final_heading_deg=90,
        position_tolerance_cm=15,
        heading_tolerance_deg=8,
    )
)
navigation.start_navigation()
```

倒车默认关闭。`NavigationConfig(allow_reverse=True)` 开启后，Hybrid A* 才会生成
倒车运动基元；倒车和换挡具有额外规划代价，仍优先选择前进路线。实际前进/倒车
切换时先停车 `0.25 s`，不会直接反向输出速度。阿克曼车辆不能原地调整最终朝向；
目标位置周围空间不足时，状态会进入 `BLOCKED`，不会把“位置已到但朝向错误”误报为
到达。

`OccupancyGrid` 使用厘米单位，单元格为 `0` 自由、`100` 障碍、`-1` 未知。未知
区域默认禁止进入，地图外始终视为障碍。碰撞检查使用随航向旋转的完整
`230 × 145 mm` 矩形并附加默认 `20 mm` 安全余量，不把车辆简化成质点。
`OccupancyGrid.from_obstacle_points()` 会把给定边界内所有非击中单元当成自由空间，
只适用于调用方明确确认整个边界为已知空间；不能仅凭雷达“没有击中”就推断自由。

安全行为：设置目标后还必须显式调用 `start_navigation()`；定位超过 `0.5 s` 未更新、
没有地图、无可行路径、已确认路径偏差过大、暂停、取消、关闭或控制异常时立即停车并
回中。雷达地图刷新后会先对尚未走完的路径做完整矩形碰撞复查：新障碍不影响剩余路径
时保留原路径连续行驶，只有路径被阻断时才停车重规划。状态可通过 `Navigation.state`
或 `on_state_changed` 回调读取。

闭环结构参考了无人机工程
`FlightController/Components/LDRadar_Driver.py` 的雷达位姿更新事件与有效性处理，以及
`FlightController/Solutions/Navigation.py` 的“新位姿驱动反馈、定位陈旧即输出零控制”
思路；无人机是全向 XY/Yaw PID，小车不能横移或原地转向，因此车端没有照搬其输出，
而是把反馈投影为阿克曼允许的前进速度和前轮转角。连续相对位姿仍由本项目已有的
SVD-ICP 与墙线有限增益纠漂提供。

## 正式主程序

文件：`main.py`

正常行驶速度集中在 `main.py` 顶部的 `NAVIGATION_CRUISE_SPEED_CM_S`，单位为
`cm/s`（例如 `50.0` 等于 `0.5 m/s`），允许范围为 `0～100 cm/s`。主程序只在把
速度传给底层驱动控制器时转换为 `mm/s`，并将正式 main 所属后轮驱动器的单轮限幅
自动设置为 `max(30, 1.20 × 巡航速度) cm/s`，为阿克曼弯道外侧轮保留余量。例如
`50 cm/s` 巡航对应 `60 cm/s` 单轮限幅。底层通用组件的默认限幅仍为 `30 cm/s`；
规划和驱动层仍会拒绝超出本次配置限幅的整条命令，不会静默缩放。

正式 main 的倒车开关位于同一区域：`NAVIGATION_ALLOW_REVERSE = True`。当前默认允许
Hybrid A* 规划倒车；前进/倒车切换仍会先停车至少 `0.25 s`。临时启动时可用
`--no-reverse` 禁止倒车，`--allow-reverse` 可显式开启。

倒车巡航速度也独立放在 `main.py` 顶部：`NAVIGATION_REVERSE_SPEED_CM_S = 15.0`，
单位和允许范围与前进巡航速度相同。该值是实际倒车速度上限，即使设置得低于接近目标
速度也不会被控制器重新抬高；正式 main 的单轮限幅会按前进/倒车两者中较大的速度预留
20% 阿克曼外侧轮余量。

当前正式 main 的前进巡航速度为 `30 cm/s`。距离目标 `60 cm` 内开始分段减速，
接近速度下限为 `8 cm/s`；倒车恢复上限为 `15 cm/s`。Pure Pursuit 的前视距离为
`20～50 cm`，终点前视目标不会再沿末段切线延伸到目标坐标之外。

如果车辆越过目标，Navigation 会连续两个雷达位姿确认越点，先停车并保留同一个
`NavigationGoal`，然后清除旧路径、从当前位置重新规划；允许倒车时规划器可以选择
倒车返回。预测前方轨迹被墙体或场地边界阻断、但当前车身仍处于安全位置时也采用同一
恢复流程。单次任务最多自动恢复三次，超过次数或当前车身已经碰撞才进入 `BLOCKED`。

启动阶段车辆必须保持静止。主程序先只打开 D500，收集完整圆周点云并用 RANSAC/PCA
拟合矩形场地的四条边。拟合成功后，会把点云、矩形边界、墙线纠漂参考和后续 ICP
位姿统一重基准到本次启动的车体坐标系：启动时后轴中心为 `(0, 0) cm`，启动时车头
方向为 `0°`，地图 `+X` 指向启动车头，`+Y` 位于车头左侧，航向俯视逆时针为正并
归一化到 `0～359°`。即使车头与矩形墙边存在小夹角，也不会只修改显示角度；旋转后的
矩形多边形之外仍全部标为障碍。矩形未可靠拟合前不会接受任务，也不会启动电机控制。

拟合成功后，主程序把矩形内部作为已知场地，矩形外（包括地图边距）全部标为障碍，
再以雷达击中点补充内部障碍。运行中由 ICP 提供连续相对位姿，并用启动时得到的墙线
参考周期性纠漂。该建图策略假定场地是静态矩形；动态障碍点当前不会自动衰减。

Navigation 使用独立的可信雷达地图：只有 ICP 残差、相邻位姿跳变和完整车身边界均通过
门限的定位才会刷新导航和地图；墙线纠偏尝试失败的扫描不会写入可信图。每次地图刷新及
提交新目标前都会清除当前实体车身范围内的自反射和历史漂移点，避免下一任务错误报告
“起始车身已占据”。异常位姿仍保留在原始雷达图和日志中用于诊断，超过定位陈旧时间后
Navigation 按原有安全逻辑停车。`BLOCKED` 日志会同时记录车身四角、附近占据格、地图
版本、可信位姿年龄和最近拒绝原因。

雷达安装偏移以**后轴中心**为车体原点，通过以下参数提供；三个默认值 `0` 只适用于
雷达测量原点确实位于后轴中心且朝向与车头完全一致的安装，实车运行前应填入测量值：

```bash
export GROUND_STATION_HMAC_KEY_HEX="至少32个十六进制字符的共享密钥"
sudo -E python3 code/main.py \
  --radar-x-cm <雷达在后轴中心前方的厘米数> \
  --radar-y-cm <雷达在后轴中心左侧的厘米数> \
  --radar-yaw-cw-deg <雷达相对车头顺时针安装角>
```

若只通过 SSH 终端控制，可以不设置 `GROUND_STATION_HMAC_KEY_HEX`；此时 HC-14 命令
入口保持关闭，终端入口照常工作。程序必须运行在 TTY 中，例如进入上位机后运行：

```bash
cd /home/radxa/car
sudo -E python3 main.py
```

建图成功后终端会显示 `=== 建图完成，Navigation 已就绪 ===`、启动位姿和旋转后的
场地四角。提示符下直接输入：

```text
200 50          # 前往 x=200cm, y=50cm，不限定最终车头
200 50 90       # 前往同一点，并最终对齐到逆时针 90°
status           # 当前定位、导航状态和原因
stop             # 立即停车回中并取消任务
help             # 显示简要帮助
quit             # 停车并退出 main
```

`x/y` 单位为厘米，可为小数；可选角度必须是 `0～359` 的整数，`360`、小数角度和场地
多边形外目标都会被拒绝。同一时间只执行一个任务；Navigation 仍使用 Hybrid A* 自主
规划，并在行驶中用每个有效雷达位姿持续修正横向误差、航向误差、转角和速度。

每次任务到达、失败或阻塞停车后，main 会清除该次目标并自动回到可接收下一目标的
`IDLE` 状态。启动时建立的矩形地图、雷达累计定位和坐标原点均保留：下一条命令仍使用
同一次启动的 `(0,0,0°)` 坐标系，并从小车任务结束时的当前位姿重新规划，不会把当前
位置重设为原点，也不会重新建图。SSH 会提示“可继续输入下一目标”；HC-14 收到终态
ACK 后同样可以发送下一条新的 `(session, seq)` 坐标任务。

正式 main 当前默认允许倒车；顶部 `NAVIGATION_ALLOW_REVERSE` 是默认开关，也可在
启动时用 `--no-reverse` 临时关闭。标定默认使用最近 3 个完整圆周、超时 30 秒，可用
`--startup-scans` 和 `--calibration-timeout` 调整。
退出信号、标定失败、定位/规划失败和远端停止都会触发停车回中。

### 运行日志

主程序每次启动都会创建 `main.py` 同级的 `logs/car-main.log`。本地路径为
`C:\Users\TZDEZACR\Desktop\cccccar\car\code\logs\car-main.log`，部署到 ROCK 5A 后为
`/home/radxa/car/logs/car-main.log`。文件始终记录 DEBUG 级详细诊断，终端仍由
`--log-level` 控制，默认只显示 INFO 及以上；也可用 `--log-dir` 或环境变量
`CAR_LOG_DIR` 覆盖目录。

日志包括启动参数（不含 HMAC 密钥）、矩形拟合结果、每圈雷达的 ICP/墙线门限与位姿、
地图刷新、规划路径摘要，以及每个运动控制周期的横向/航向误差、定位降速系数、限幅
前后舵角、PWM、左右后轮目标和实际下发的 C10B `Vx/Vz`。它不逐点转储点云，也不记录
HMAC 密钥。单文件达到 `20 MiB` 后轮转，保留 `10` 个历史文件；`logs/` 已写入
`.gitignore`，运行记录不会进入 Git。

板端实时查看：

```bash
tail -f /home/radxa/car/logs/car-main.log
```

### 地面站坐标命令

`components/navigation_protocol.py` 复用地面站既有 GroundStationLink V2 的
`AA 22` 元数据、校验和及 HMAC。HC-14 串口组件会在链路层自动增加/移除
`BB 33 | length:u8` 外层。共享密钥只从环境变量
`GROUND_STATION_HMAC_KEY_HEX` 读取，不得写入源码或日志。未设置密钥时不会以不鉴权方式
打开 HC-14，而是仅关闭无线命令入口。

新增车辆命令号为 `NAVIGATE_TO = 0x20`，其业务 payload 为：

```text
command_id:u8 = 0x20
flags:u8             bit0=1 表示携带最终航向
x_cm:i32 LE
y_cm:i32 LE
[heading_centideg:u16 LE]   可选，0..35999，即 0.00..359.99°，俯视逆时针为正
```

不带航向时只要求到达目标坐标附近；带航向时只有位置和朝向同时满足容差才返回
`COMPLETED`。重复的 `(session, seq)` 命令不会重复发车。已有 `STOP_MISSION = 5`
用于取消当前导航。主程序依次返回 V2 `RECEIVED`、`ACCEPTED/REJECTED`，任务结束后
返回 `COMPLETED/FAILED`。

发送的 `x/y/heading` 必须属于本次启动建立的车头零度场地坐标系。主程序不会猜测或自动
转换无人机另一个原点/方向的坐标；若无人机内部使用不同地图，发送端必须先做已标定的
SE(2) 坐标变换。

发送端可以复用 `pack_navigation_command(NavigationGoal(...))` 生成完整内部
`AA 22` 帧。当前 `ground_station` 原代码尚无 `0x20` 坐标命令，地面站/无人机发送端
必须按上述 payload 增加该命令后才能下发坐标；车端接收和 ACK 已完成。
