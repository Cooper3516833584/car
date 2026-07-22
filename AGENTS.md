ROCK 5A ssh地址 radxa@192.168.31.224  密码11223344
所有代码应位于\car\code内

## 代码目录约定

- 正式上位机入口为 `code/main.py`。
- 可复用硬件组件统一放在 `code/components/`。
- 单元测试、实车联调脚本、临时程序、一次性配置工具和试验补丁统一放在 `code/test/`，不得混入正式组件目录。
- 正式驱动组件为 `code/components/rear_motor.py`（后轮）、`code/components/steering_servo.py`（前轮舵机）和 `code/components/ackermann_drive.py`（车速/偏航统一控制），详细接口、限制及资料依据见 `code/README.md`。
- HC-14正式串口组件为 `code/components/serial_communication.py`。它只负责 `115200 8N1`、DTR/RTS清除、自动重连和 `BB 33`桥封装；上层收发的是完整内部 `AA 22`帧，业务解析、HMAC、ACK及车辆控制不得塞入串口组件。

## 已验证的小车连接与控制

- ROCK 5A 通过 USB 连接 WHEELTEC L150/C10B 驱动板，设备为 `/dev/ttyACM0`，波特率 `115200`。
- C10B 接收 11 字节速度帧：`7B 00 00 VX_H VX_L 00 00 VZ_H VZ_L BCC 7D`，其中 BCC 是前 9 字节异或；例如直行 `100 mm/s`：`7B 00 00 00 64 00 00 00 00 1F 7D`。
- 电机使能由驱动板 `KEY2` 控制。串口遥测帧第 2 字节为 `00` 时已使能，`01` 时电机被关闭；测试前先确认其为 `00`。
- 左后轮接左侧带编码器电机接口，右后轮接右侧带编码器电机接口。ROCK 5A 上的低速、自动停止测试程序：`/home/radxa/wheel_test.py`；本地源文件为 `code/test/wheel_test.py`。
- C10B 阿克曼固件只接收 `Vx/Vz`，不接收原始左右轮目标。正式组件可将可实现的左右轮目标逆变换为 `Vx=(left+right)/2`、`Vz=(right-left)/0.164`；固件最小转弯半径为 `0.350 m`，因此单轮转动、左右反转和原地旋转必须拒绝，不能假报成功。

## 前轮转向舵机（阿克曼）

- 舵机信号线接 ROCK 5A 40Pin 物理 `Pin 23`（`PWM0_M2`）；舵机地线必须与 ROCK 5A 共地，舵机电源使用小车的稳定 5V 电源。
- 已通过官方设备树覆盖项 `rk3588-pwm0-m2` 启用 Pin 23 的 PWM0；该覆盖项首次启用后需要重启 ROCK 5A。当前 `pwmchip0` 对应 `/sys/devices/platform/fd8b0000.pwm`。
- 舵机使用 50 Hz（周期 `20,000,000 ns`）、正常极性 PWM。本车重新标定后的回中脉宽为 `1580 us`。
- WHEELTEC L150 源码 `CONTROL/control.c` 中已给出厂家转向角到 PWM 的标定：
  `PWM_us = clamp(1500 + 640.62 * (-0.628*theta^3 + 1.269*theta^2 - 1.772*theta + 0.001), 800, 2200)`。
  厂家曲线角度限幅为 `-0.49 .. +0.32 rad`；厂家曲线整体增加 `79 us` 以匹配本车
  实测中位。2026-07-22 实车日志与前轮目测确认本车舵机/连杆方向和厂家车辆方向相反，
  必须使用 `calibration_theta=-vehicle_theta`，不可再当作待确认项。逻辑车辆范围因此为
  `-0.32 .. +0.49 rad`，负数右转、正数左转。
- ROCK 5A 程序：`/home/radxa/steering_servo_test.py`；本地源文件：`code/test/steering_servo_test.py`。可执行 `sudo python3 /home/radxa/steering_servo_test.py --sweep` 做小幅左右回中测试。
- 正式舵机组件使用厂家三次标定表、反向安装修正和本车 `+79 us` 中位修正；正角为
  左偏航、负角为右偏航，逻辑范围 `-0.32 .. +0.49 rad`，当前对应
  `-0.12/0/+0.12 rad -> 1454/1580/1728 us`。
- 前后联动必须区分两套参数：实体阿克曼几何使用实测左右轮中心距 `117.1 mm`、轴距
  `142.5 mm`；C10B 串口逆变换单独使用固件编译值 `164 mm`，即
  `Vz=(right-left)/0.164`。不得把 `117.1 mm` 传给驱动板协议换算，否则固件产生的
  后轮差速会约放大 `164/117.1≈1.40` 倍，无法与前轮适配。运动时默认启用后轮差速
  联动；实体转弯半径仍不得小于 `350 mm`，高速大转角若导致外侧轮超过速度限幅，
  必须拒绝整条命令并要求上层降速。

## D500 雷达接线

按用户提供的 D500 接线图连接至 ROCK 5A 40Pin：

- D500 `P5V` -> 物理 `Pin 2`（5V）。
- D500 `GND` -> 物理 `Pin 20`（GND）。
- D500 `TX` -> 物理 `Pin 21`（`UART6_RX_M1`）。
- D500 `PWM` -> 物理 `Pin 25`（GND）。该线按接线图接地，并非 ROCK 5A 的 PWM 输出。
- 2026-07-22 已通过官方 `rsetup` 启用 `rk3588-uart6-m1.dtbo` 并重启；当前
  `/boot/extlinux/extlinux.conf` 同时保留 `rk3588-pwm0-m2.dtbo` 与
  `rk3588-uart6-m1.dtbo`，设备 `/dev/ttyS6` 已出现，属组 `dialout`。修改前启动配置
  备份为 `/boot/extlinux/extlinux.conf.codex-before-uart6-20260722`。
- 只读探针 `code/test/d500_uart_probe.py` 可验证 UART、`54 2C` 帧、CRC、完整圆周及
  可选矩形拟合。2026-07-22 实测 6 秒收到 `2498` 个有效包、`59` 个完整圆周、
  `0` 个 CRC 错误，启动矩形拟合成功；探针不访问驱动板、舵机或电机。

## D500 雷达正式组件与坐标约定

- 正式组件位于 `code/components/radar_driver.py`，默认从 `/dev/ttyS6`
  （Pin 21 / UART6_RX_M1）以 `230400 8N1` 只读方式接收；使用前须启用
  `UART6-M1` 设备树覆盖。组件不会写雷达 UART、不会控制雷达 PWM，也不会操作驱动板。
- D500/STL-19P 帧固定为 47 字节：`54 2C`、转速、起始角、12 组距离/置信度、
  结束角、时间戳、CRC-8/0x4D。解析器必须校验 CRC 并在坏帧后重新同步；建图只使用
  角度过零后的完整圆周，启动残缺首圈必须丢弃。
- 雷达定位参考无人机工程的相邻点云 SVD-ICP，但车端增加匹配距离、单步位移、
  航向变化和误差门限。正式运行环境需要 `numpy`；定位拒绝时不得写入该帧地图。
- 雷达最终定位为“连续 ICP + 可选矩形墙线周期纠漂”。墙线法参考
  `python_sdk/former_code/2026_radar.py` 及其调用的 `radar_resolve_rt_pose()`，但车端
  使用 ICP 预测辅助关联后墙/右墙，并通过点数、线长、RMS、轴向角、两墙一致性及
  相对 ICP 残差门限后才允许有限增益回写里程计；墙线失败时继续使用 ICP。
- 雷达定位和地图的统一坐标约定必须与无人机一致：`+X` 前、`+Y` 左、厘米，
  航向角以度表示且**顺时针/右转为正**。车辆控制中的转向角却是左转为正，二者
  符号相反；由车辆模型提供航向增量时必须先转换符号。
- `RadarMount` 只负责雷达传感器坐标到车体坐标的安装偏航和平移；
  `DroneGlobalAlignment.from_reference()` 负责小车局部地图到无人机全局地图的固定
  SE(2) 旋转和平移。必须用同一物理时刻的“小车局部参考位姿 + 无人机全局参考位姿”
  标定，禁止猜测或写死车头与无人机方向差。
- 未提供 `DroneGlobalAlignment` 或未调用 `set_global_reference()` 时可以输出小车局部
  里程计，但不得生成无人机全局地图；全局标定后，位姿和所有点云必须经过同一个
  变换再发布。
- 墙线融合默认关闭。启用前必须通过 `RectangularWallReference` 明确墙体局部坐标到
  无人机全局坐标的变换；墙体局部约定后墙 `x=0`、右墙 `y=0`、`+X/+Y` 均指向场内。
  禁止猜测墙角全局位置或场地方向。默认每 5 个完整圆周尝试一次，位置/航向增益为
  `0.20/0.15`，默认关联范围 `45 cm`；它只纠正缓慢漂移，不用于严重失配后的全局
  重定位。接受纠正后必须回写 `RadarOdometry.pose`，建图和 Navigation 使用同一姿态。
- 本组件资料依据：
  `C:\Users\TZDEZACR\Desktop\DDDDDrone_Cloned\python_sdk\FlightController\Components\LDRadar_Driver.py`、
  `LDRadar_Resolver.py`、`Utils.py`，
  `FlightController\Solutions\Radar_SLAM.py`，以及明确无人机全局旋转矩阵的
  `python_sdk\warehouse_radar_localizer.py`；墙线纠漂还参考
  `python_sdk\former_code\2026_radar.py`。

## Navigation 自主导航组件

- 正式组件为 `code/components/navigation.py`；单元测试为
  `code/test/test_navigation.py`。不得在 Navigation 内重复实现舵机 PWM 或 C10B
  串口协议，只能调用 `AckermannDrive`。
- 已确认实车尺寸：车轮厚 `26.4 mm`，左右轮外侧总宽 `143.5 mm`，因此左右轮中心距
  为 `117.1 mm`；前后轴距 `142.5 mm`；碰撞车身按 `230 × 145 mm` 矩形处理。
  驱动固件约 `350 mm` 最小转弯半径仍是更高优先级的下限。由于舵机机械范围不对称，
  规划器分别计算左右实际可用最小半径，不能假定左右曲率完全相同。
- Navigation 的 `NavigationPose` 原点是后轴中心。暂按车身前后悬对称处理，车身矩形
  中心位于后轴前方 `71.25 mm`。雷达 `RadarMount` 的安装偏移必须也以后轴中心为
  车体原点；若测得不对称前后悬，应更新 `rear_axle_to_body_center_cm`。
- Navigation 地图坐标：地图 `+X` 为无人机 `0°`，`+Y` 为俯视时 `+X` 左侧，航向
  俯视逆时针为正并归一化为 `0～359°`。雷达/无人机位姿航向顺时针为正，所以入口
  必须使用 `navigation_heading=(-radar_yaw_cw)%360`；车辆转向正数左转与 Navigation
  一致，不得再次反号。
- 路径规划使用带旋转矩形碰撞检查的 Hybrid A*，跟踪使用 Pure Pursuit。最终航向
  可选；阿克曼车不能原地转向，位置达到但朝向不满足时不得报告 `ARRIVED`。
- 带最终航向的前进任务应先尝试使用左右均可实现的保守半径生成 Dubins 解析连接，
  且必须对完整曲线逐段执行旋转矩形碰撞检查；解析连接受阻才回退 Hybrid A*。Hybrid
  A* 单次搜索默认限时 `5 s`，规划必须响应停止、取消、暂停和退出，禁止长期占满 CPU
  或在任务取消后突然发车。
- 倒车由 `NavigationConfig.allow_reverse` 控制，默认 `False`。关闭时规划器不得生成
  倒车；开启后倒车和换挡需要额外代价，前进/倒车切换必须先停车至少 `0.25 s`。
- 占据栅格值约定为 `0` 自由、`100` 障碍、`-1` 未知；未知默认不可通行，地图外
  一律视为障碍。雷达击中点地图中的“未击中”不等于已确认自由，调用
  `from_obstacle_points()` 时必须由上层明确限定已知自由边界。
- 设置目标不会自动发车，必须再调用 `start_navigation()`。定位陈旧、地图缺失、
  路径不可达、偏离路径、暂停、取消、关闭或异常时必须停车回中；地图、目标或定位在
  规划期间改变时，旧规划结果必须丢弃。
- 行驶控制为“Pure Pursuit 曲率前馈 + 雷达位姿横向/航向反馈”闭环，不是固定速度或
  时间开环。每个通过 ICP/墙线门限的完整雷达圆周都会立即唤醒 Navigation，重新计算
  舵角和速度；拒绝的雷达圈不得刷新定位时间，超过 `0.5 s` 无有效位姿必须停车回中。
- 默认横向/航向反馈增益 `0.35/0.65`，转角变化率上限 `1.2 rad/s`。横向或航向偏差
  增大时主动降速；ICP 平均残差从 `4 cm` 开始降速，到 `10 cm` 时速度系数降为
  `0.40`。超过 `35 cm` 的路径偏差需由 3 个不同雷达位姿确认，超过 `60 cm` 立即
  停车重规划，禁止把同一雷达位姿在 20 Hz 控制循环中重复计数。
- Pure Pursuit 曲率叠加横向/航向反馈后，最终舵角必须再次按实体左右最小转弯半径
  限幅。反转舵机映射后右侧机械边界为约 `-0.32 rad`（实体半径约 `489 mm`）；左侧
  虽可机械到 `+0.49 rad`，运动时必须收紧到约 `+0.336 rad` 才满足 `350 mm` 下限。
- 雷达占据图更新后必须复查剩余路径的旋转矩形碰撞；新障碍不影响路径时保留路径和
  行驶连续性，路径被阻断时才失效并重规划。路径最近点搜索进度不得因交叉路线或小幅
  墙线纠漂跳回已经走过的旧段。
- 该闭环设计参考无人机 `FlightController/Components/LDRadar_Driver.py` 的位姿更新
  事件/有效性处理和 `FlightController/Solutions/Navigation.py` 的新位姿反馈及陈旧
  定位零输出；无人机全向 XY/Yaw PID 不可直接照搬到阿克曼车，车端反馈最终只能输出
  前进/倒车速度与前轮转角。

## 地面站 HC-14 串口通信（2026-07-21）

- 地面站树莓派：`cooper@192.168.31.107`；无线模块为 CH340/HC-14，稳定设备路径为 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`，映射到 `/dev/ttyUSB0`。
- 小车 ROCK 5A 测试架新插入模块：`/dev/ttyUSB0`，稳定路径为 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`，USB 标识为 CH340（`1a86_USB_Serial`）。小车驱动板仍固定使用 `/dev/ttyACM0`，两者不可混用。
- HC-14 透明传输必须使用 `115200 8N1`，关闭流控并在打开串口前后明确清除 `DTR`、`RTS`；地面站既定无线参数为 `B115200 / C28 / S8 / +20 dBm`。
- 小车新模块最初实测为 `B9600 / C28 / S8 / +20 dBm`，因此首次透明传输双向各 3 条均为 `0/3`。2026-07-21 已发送 `AT+B115200`，回读确认其当前参数为 `B115200 / C28 / S8 / +20 dBm`，与地面站一致。
- 参数统一后用无控制含义的编号 ASCII 探针做双向各 5 条低频实测：地面站 -> 小车 `5/5`，小车 -> 地面站 `5/5`，HC-14 双向透明链路已确认可用。
- 小车测试架的 `RTS=True` 会将 HC-14 的 `KEY/SET` 拉入 AT 模式；`RTS=False` 为透明传输模式。HC-14 AT 命令必须作为纯 ASCII 发送，**不得附加 CR/LF**，否则会额外返回 `ORDER ERROR`。修改波特率后退出 AT 模式至少等待 `250 ms` 再重入或透传。
- 未经用户明确授权，不得发送会修改波特率、信道、空中速率、功率或恢复出厂设置的 HC-14 AT 命令。优先检查模块 `KEY/SET` 状态、供电、天线、两端配对/信道和本地 UART 参数。
- 本地双向测试程序：`code/test/hc14_ground_link_test.py`；只读 AT 查询程序：`code/test/hc14_at_probe.py`；经明确授权后使用的单项波特率设置及回读工具：`code/test/hc14_set_baud.py`。测试程序只发送低频 ASCII 探针，不发送小车、飞控或任务控制帧。
- 本节资料依据：`C:\Users\TZDEZACR\Desktop\ground_station\details.md`、地面站项目的既有 HC-14 联调记录，以及 HC-14 V1.0 厂商用户手册；最终参数和 `5/5` 双向结果均为本车与当前地面站的现场实测。

## 正式 main 启动标定与坐标命令（2026-07-22）

- 正常自主导航巡航速度集中定义在 `code/main.py` 顶部的
  `NAVIGATION_CRUISE_SPEED_CM_S`，单位 `cm/s`；调整速度只修改此处，主程序传给底层
  控制器时乘以 `10` 转为 `mm/s`。正式 main 按
  `max(30, 1.20 × 巡航速度) cm/s` 自动配置单轮限幅，为阿克曼外侧轮保留余量；巡航
  速度只允许 `0～100 cm/s`，超出范围必须在启动时拒绝。底层通用组件默认单轮限幅
  仍为 `30 cm/s`，任何超限命令仍须整条拒绝，禁止静默缩放。
- 正式 main 的倒车默认开关是 `code/main.py` 顶部的
  `NAVIGATION_ALLOW_REVERSE`，当前为 `True`；`--no-reverse` 可在单次启动中临时关闭，
  `--allow-reverse` 可显式开启。倒车换挡仍必须先停车至少 `0.25 s`。
- `code/main.py` 启动时车辆必须静止；先收集 D500 完整圆周并拟合矩形场地四边，再将
  点云、墙线参考、矩形边界和后续 ICP 位姿一起重基准：启动后轴中心为 `(0,0) cm`，
  启动车头为 `0°`，`+X` 指向启动车头、`+Y` 在其左侧，Navigation 航向俯视逆时针
  为正。墙边与车头存在夹角时必须保留旋转后的矩形多边形，禁止只把显示角度清零。
- 矩形可靠拟合完成前不得打开 HC-14 任务入口或启动电机控制。拟合矩形之外的网格
  必须全部视为障碍，禁止规划器从墙外绕行；目标坐标也必须位于拟合边界内。
- 雷达安装参数 `--radar-x-cm/--radar-y-cm/--radar-yaw-cw-deg` 都以后轴中心和车头
  为基准；默认全零仅适用于测量原点与后轴中心重合且雷达零角与车头一致的安装。
- 坐标任务沿用 GroundStationLink V2 鉴权帧，新增业务命令 `NAVIGATE_TO=0x20`：
  `command_id:u8, flags:u8, x_cm:i32LE, y_cm:i32LE, [heading_centideg:u16LE]`；flags bit0
  表示可选最终航向，角度范围 `0..35999`、单位 `0.01°`、俯视逆时针为正。
  `STOP_MISSION=5` 取消任务；重复 `(session,seq)` 不得重复执行。
- 命令中的坐标/航向必须已经位于本次 main 启动建立的场地坐标系；若无人机使用其他
  原点或方向，发送端必须先应用已标定的 SE(2) 变换，车端禁止猜测该变换。
- `GROUND_STATION_HMAC_KEY_HEX` 至少 16 字节，只能从环境变量加载，禁止硬编码。
  当前地面站原代码没有 `0x20`，发送端需按 `code/README.md` 的格式同步扩展。
- SSH TTY 为正式本地任务入口。建图完成后打印明确提示和场地四角，接受
  `x_cm y_cm [heading_deg]`；`x/y` 单位 cm，可选航向必须是 `0..359` 整数。还必须
  支持 `status/stop/help/quit`，同一时间只能执行一个任务，场地多边形外目标必须拒绝。
  没有 HMAC 环境变量时只关闭 HC-14，严禁降级为未鉴权无线控制，SSH 入口仍可使用。
- 主程序当前默认允许倒车，可用 `--no-reverse` 临时关闭；退出、停止、定位丢失、规划
  失败或异常均必须停车回中。当前运行地图假定静态矩形，累计动态障碍不自动衰减。
- 每次任务进入 `ARRIVED/FAILED/BLOCKED` 后，main 必须清除本次目标并回到可接收下一
  坐标的 `IDLE`；只清任务状态，必须保留启动矩形地图、雷达累计位姿和本次启动定义的
  `(0,0,0°)` 原点。下一任务从当前位姿重新规划，禁止重新建图或把结束点重设为原点。
- `code/main.py` 的详细运行日志固定默认写入其同级 `code/logs/car-main.log`；板端对应
  `/home/radxa/car/logs/car-main.log`。文件级别始终为 DEBUG，单文件 `20 MiB`、保留
  `10` 个轮转备份，终端级别仍由 `--log-level` 控制。日志须包含雷达 ICP/墙线、位姿、
  地图刷新、规划摘要及控制周期的误差/舵角/PWM/后轮/C10B 输出，但不得记录 HMAC 密钥
  或逐点转储整圈点云。`code/logs/` 必须保持在 `.gitignore` 中。
