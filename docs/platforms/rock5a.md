# ROCK 5A 当前车辆接线（当前实车验证）

本文档只记录**当前这辆车**（Cooper ROCK 5A + WHEELTEC L150）的验证过接线。
换主控板请参考 `docs/HARDWARE_PORTING.md`；这些信息**不会**再出现在业务代码
的报错提示里。

## Steering（前轮舵机）

| 项目 | 值 |
|---|---|
| 物理 Pin | 23 |
| 引脚功能 | `PWM0_M2` |
| 设备树 overlay | `rk3588-pwm0-m2` |
| PWM 设备 | `fd8b0000.pwm`（`pwmchip0`） |
| 频率/周期 | 50 Hz / `20,000,000 ns`，极性 normal |
| 回中脉宽（实测） | `1580 us` |

启用 overlay 后需要重启；`enable_pwm0_m2.sh` 位于 `code/test/`。

## D500 雷达

| 项目 | 值 |
|---|---|
| D500 `TX` | 物理 Pin 21（`UART6_RX_M1`） |
| D500 `P5V` | 物理 Pin 2（5V） |
| D500 `GND` | 物理 Pin 20（GND） |
| D500 `PWM` | 物理 Pin 25（GND，该线按接线图接地，不是 ROCK 5A PWM 输出） |
| 设备树 overlay | `rk3588-uart6-m1.dtbo` |
| 设备 | `/dev/ttyS6`，属组 `dialout` |
| 波特率 | `230400 8N1`，只读 |

`rsetup` 启用 overlay 后重启；当前 `/boot/extlinux/extlinux.conf` 同时保留
`rk3588-pwm0-m2.dtbo` 与 `rk3588-uart6-m1.dtbo`。修改前备份：
`/boot/extlinux/extlinux.conf.codex-before-uart6-20260722`。

## Alarm（声光报警）

| 项目 | 值 |
|---|---|
| 物理 Pin | 11 |
| 引脚功能 | `GPIO4_B3`（bank `gpio4`，line offset 11，全局约 139） |
| 极性 | active low（拉低=报警开） |

开机服务 `sound-light-alarm.service` 执行 `--off --grant-group gpio`，让
`gpio` 组成员可写 `direction`/`value`。

## C10B 驱动板

- USB 设备 `/dev/ttyACM0`，`115200 8N1`。
- 电机使能由驱动板 `KEY2` 控制；串口遥测帧第 2 字节 `00`=已使能。
- 左后轮接左侧带编码器电机接口，右后轮接右侧带编码器电机接口。

## HC-14 无线串口

- 设备 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`（CH340）。
- `115200 8N1`，关闭流控，打开串口前后清除 DTR/RTS。
- 参数 `B115200 / C28 / S8 / +20 dBm`；AT 命令必须纯 ASCII、不带 CR/LF。
