# Drive Protocol and Hardware Notes

> Extracted from the previous long README so the repository front page stays a
> navigation guide. The content is unchanged except for heading level and this note.

Contents: C10B battery monitoring, rear-motor protocol, front steering servo,
front/rear linkage, GPIO alarm wiring, HC-14 serial component.

## C10B 电池低压报警

`components/battery_voltage_monitor.py` 以只读方式监听 C10B 的 `/dev/ttyACM0`
遥测帧，不写入串口、也不清空接收缓冲，因此可与后轮驱动的命令写入并行运行。它按
厂商 L150 固件的 24 字节 `7B ... BCC 7D` 帧校验数据，并从第 20--21 字节复原电池
电压。服务每 `2 s` 读取一次（`0.5 Hz`）；连续 5 次严格低于 `11.00 V` 时，GPIO4_B3
的低电平声光报警器持续开启。任意一次有效读数达到 `11.00 V` 或更高会复位计数并关闭
报警。

开机服务单元在 `code/test/battery-voltage-monitor.service`。板端安装时先确保
`sound-light-alarm.service` 已安装，再执行：

```bash
sudo cp /home/radxa/car/code/test/battery-voltage-monitor.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now battery-voltage-monitor.service
```

可用 `sudo systemctl status battery-voltage-monitor.service` 与
`journalctl -u battery-voltage-monitor.service -f` 查看读取结果。厂商源码将“百分伏值
乘 1000”写入 16 位字段；组件会按该溢出编码反推 `6--18 V` 内唯一值，现场首次启用时
仍应使用万用表核对一次读数。

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

常规阿克曼行驶仍强制最小转弯半径 `350 mm`。3×5 相邻格任务是唯一显式例外：构造
`RearMotorDriver(allow_in_place_rotation=True)` 后，可以用 `Vx=0`、左右后轮等速反向
原地旋转；该模式必须先停车、将前轮回正并使用雷达航向闭环。默认值仍为 `False`，
普通 Navigation 不会隐式产生单轮、左右反转或原地旋转命令。

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

## GPIO4_B3 声光报警器

低电平触发的声光报警器接 ROCK 5A `GPIO4_B3`（40Pin 物理 Pin 11）。组件会在运行时从
`gpio4` 的 sysfs base 解析 line 11，当前板端对应全局 GPIO 139；初始化时使用
`direction=high` 原子地切换为输出高电平，避免切换方向时产生短暂的低电平报警。

```python
from components import SoundLightAlarm

alarm = SoundLightAlarm().initialize()  # 高电平，关闭报警
alarm.on()                              # 低电平，开启声光报警
alarm.off()                             # 高电平，关闭声光报警
```

板端安装了 `sound-light-alarm.service`，开机自动执行 `--off --grant-group gpio`，保持
GPIO4_B3 为高电平，并允许 `gpio` 组中的项目进程随时调用组件。服务单元的仓库副本位于
`code/test/sound-light-alarm.service`。

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

