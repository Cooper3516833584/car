# Ackermann Ground Robot

> 小车上位机代码结构 / on-vehicle software for the D-task Ackermann vehicle.

Ackermann ground robot for the "D-task" competition, built around a ROCK 5A-class
host, a WHEELTEC C10B rear-motor driver, a front steering servo, a D500 LiDAR and
a V4L2 camera. The two production entries share one mission core and are fully
configured through TOML profiles.

| Item | Value |
|---|---|
| Drive | Rear C10B motor driver + front steering servo (Ackermann) |
| Perception | D500 LiDAR (`radar_driver`), USB camera line following |
| Host | ROCK 5A (current profile: `configs/cooper_rock5a_l150.toml`) |
| Config | TOML profiles in `configs/`, loaded by `code/config/` |
| Radio | HC-14 FleetBus link to the ground station |
| Optional | ROS 2 bridge and Nav2 launch files under `code/ros2_ws/` |

**Architecture:** see [docs/SYSTEM_ARCHITECTURE.md](docs/SYSTEM_ARCHITECTURE.md) for
the system panorama, mission data flow and a component-to-source evidence table.

**Validation:** the code paths are documented from source; this README does not
claim a specific competition result. Hardware values (wheelbase, track width,
servo center) are calibration data, not universal constants. The ECDC variant is
kept on the separate `competition/ecdc-2026` branch and is not this `main`.

正式比赛入口为 `code/main_task1.py`（任务1）与 `code/main_task2.py`（任务2），
两者共用比赛核心 `code/components/radar_camera_line_following.py`。
可复用硬件模块放在 `code/components`，硬件抽象层在 `code/hal`，
统一 TOML 配置在 `configs/` 与 `code/config/`，所有测试放在 `code/test`。

## Hardware and drive protocol

The C10B frame format, battery monitoring, steering calibration, front/rear
linkage, GPIO alarm wiring and HC-14 serial details have moved to
[docs/DRIVE_PROTOCOL.md](docs/DRIVE_PROTOCOL.md). The sections below keep the
competition-level overview; deep hardware material lives under `docs/`.

## 🚀 其他学校快速开始

本仓库是**同一道比赛题**的配置化版本：换车不需要改主程序和算法，只需要复制
示例配置并修改车辆数据。

```bash
# 1. 复制示例配置，得到自己学校的配置
cp configs/car.example.toml configs/my_car.toml

# 2. 修改车辆尺寸（[vehicle.geometry]：轴距/轮距/车身）
# 3. 修改驱动串口（[devices.motor] / [devices.radar] / [devices.hc14] / [devices.screen]）
# 4. 配置 PWM（[hardware.steering_pwm]，按新主控板改 chip_device_match）
# 5. 配置 GPIO（[hardware.alarm_gpio]，注意 active_low）
# 6. 标定舵机（[vehicle.steering]：direction_sign / center_us / 机械范围 / 曲线）
# 7. 设置雷达位置（[sensors.radar.mount]，相对后轴中心）
# 8. 标定摄像头（[sensors.camera.perspective] 与 [sensors.camera.line]）
# 9. 运行低速测试（见 docs/CALIBRATION.md 第 11 步）
# 10. 启动任务
python3 code/main_task1.py --config configs/my_car.toml
python3 code/main_task2.py --config configs/my_car.toml
```

不传 `--config` 时默认解析到 `configs/cooper_rock5a_l150.toml`（当前验证车辆），
也可用环境变量 `CAR_CONFIG=/path/to/config.toml` 指定。

详细文档：

- [docs/CONFIGURATION.md](docs/CONFIGURATION.md) — 配置结构、优先级、两套轮距
- [docs/HARDWARE_PORTING.md](docs/HARDWARE_PORTING.md) — 换主控板步骤
- [docs/CALIBRATION.md](docs/CALIBRATION.md) — 实车标定操作顺序
- [docs/platforms/rock5a.md](docs/platforms/rock5a.md) — 当前 ROCK 5A 实车接线

## 正式驱动组件

- `components/rear_motor.py`：C10B 后轮串口控制、20 Hz 刷新、超时停车。
- `components/steering_servo.py`：前轮转向舵机标定曲线（标定数据来自配置）。
- `components/ackermann_drive.py`：统一设置车速、前轮偏航方向，并可联动后轮差速。
- `components/sound_light_alarm.py`：声光报警器开关（GPIO 来自配置）。
- `components/battery_voltage_monitor.py`：C10B 串口电池遥测与低压声光报警服务。
- `components/trusted_navigation_map.py`：可信雷达位姿门限、车体自反射过滤和导航占据图。

`main_radar_camera_line_following.py` 通过 `Navigation`/`AckermannDrive` 间接使用
前后独立组件；只有维护、标定或特殊控制时才直接访问。

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
    WallLineConfig,
)

# 示例：后墙/右墙交点在无人机全局 (300, -200) cm，场地 +X 相对
# 无人机全局 +X 顺时针旋转 90°。数值必须由现场测量替换。
wall_to_global = DroneGlobalAlignment.from_reference(
    car_local_pose=Pose2D(0, 0, 0),
    drone_global_pose=Pose2D(300, -200, 90),
)
radar.enable_wall_fusion(
    RectangularWallReference(wall_to_global),
    # 先用预测车身航向把点云旋转回墙体坐标系，允许小车在转弯中定位。
    line_config=WallLineConfig(rotation_adaptation=True),
    # 车端只把墙线作为 ICP 之上的低频、小步漂移修正。
    fusion_config=WallFusionConfig.car_slow_drift(
        position_gain=0.20,
        yaw_gain=0.15,
    ),
)
```

正式 `main.py` 默认使用上述车端慢速纠漂模式。每个完整圆周仍可独立解算可见墙体对应的
绝对 X/Y 和车身航向，但默认每 `5` 圈才尝试一次墙线融合，位置/航向增益为
`0.20/0.15`；未观测到的轴继续沿用 ICP。
为适配会转弯的小车，定位器先使用 ICP 预测航向旋转点云，再从墙线剩余角度求航向修正。
它仍保留车端安全门限：连续三次墙线尝试的残差一致后才允许融合，位置/航向残差分别不得超过
`12 cm/8°`，单次修正不得超过 `1 cm/0.5°`。这些门限用于拒绝货架、路沿或车体反射形成
的平行假墙，避免单圈十几厘米的错误回写。墙线与 ICP 的二维位置残差达到 `8 cm` 时
Navigation 进入 `RELOCALIZING` 并保持停车；残差连续两次回到 `3 cm` 以内后，保留原
目标并从修正后的当前位置重新规划。

每个完整圆周仍先由 ICP 推进连续位姿。到达配置周期后，墙线定位器使用 ICP 预测把
点云转换到墙体坐标系，对后墙和右墙分别做候选距离筛选、离群点剔除和 PCA 直线拟合；
同一 `45 cm` 关联带内存在多条平行结构时，先按轴向坐标分簇，再选择离已标定墙坐标
最近且通过质量门限的线簇，禁止让点数更多的货架或路沿用整体中位数顶替真实外墙。
直线必须满足最少 `25` 点、长度、`3.5 cm` RMS、轴向角和两墙正交一致性门限。X、Y
和航向分别独立处理：某一面墙暂时被遮挡或质量不足时，只暂停对应轴。通过质量检查后，
墙距/航向还必须通过相对 ICP 预测的最大位置和航向残差门限，才按低通增益回写
`RadarOdometry.pose`。纠正后的同一位姿用于全局点云建图和 Navigation，避免定位与
地图使用两套姿态。

Navigation 每次实际下发非零车速后会通知雷达里程计“车辆运动中”；停车、规划、到达、
重定位和安全停止时则通知“车辆静止”。零速命令后的 `1 s` 制动宽限内仍正常积分真实
滑行；宽限结束后 ICP 继续做匹配和质量检查，但不再把桌腿、车身反射或点云抖动积分成
车辆位移，每个通过门限的扫描都会刷新静止参考帧。若静止时连续三圈触发 ICP 硬门限，
组件只重建参考帧而不改变车辆坐标，下一圈即可恢复，不会永久卡在 `translation/error
gate`。墙线绝对定位仍可在静止期间继续有限增益回写，因此 `RELOCALIZING` 不会被 ICP
假运动抵消。

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

前进任务会先尝试使用保守对称转弯半径生成 Dubins 解析连接：带最终朝向时使用指定
朝向，仅指定坐标时枚举可到达的终点朝向并选择最短安全曲线。整条曲线按 `2.5 cm`
间隔执行旋转车身矩形碰撞检查；解析曲线被障碍挡住时才回退 Hybrid A*。Hybrid A*
搜索期间会定期让出 CPU 给 D500 线程，单次搜索默认最多 `5 s`，并会响应停止、取消、
暂停和退出，避免复杂
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
- 坐标任务默认到达容差为 `5 cm/5°`；带最终航向的任务进入终点 `50 cm` 范围后限速
  `5 cm/s`，若连续 `3` 个不同雷达位姿同时出现至少 `6 cm` 横向偏差和转向饱和，
  会先停车再从当前位姿重新规划，不会继续驶入宽松的到达圈；
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

# 只要求抵达坐标附近。坐标相对本次启动原点，+X 向启动车头，+Y 向左。
navigation.navigate_to(500, 240)

# 或要求抵达后车头相对启动车头逆时针 90°。
navigation.navigate_to(
    500,
    240,
    90,
    position_tolerance_cm=15,
    heading_tolerance_deg=8,
)
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

底层仍保留 `set_goal()` + `start_navigation()` 两阶段接口，供需要“加载但暂不发车”的
任务控制器使用；普通坐标调用使用 `navigate_to()` 一次完成。定位超过 `0.5 s` 未更新、
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

## 可信定位与导航地图组件

文件：`components/trusted_navigation_map.py`。`TrustedNavigationMap` 封装了原先位于
`main.py` 内的可信雷达定位门限、车体自反射过滤、历史自反射清理、雷达点累计，以及
“矩形场地外全部为障碍”的占据栅格生成策略。它不直接操作电机、舵机或
`Navigation`；正式 main 先让该组件校验雷达更新，再交给 Navigation 接受位姿，最后
把通过门限的点云写入可信地图。

组件通过 `TrustedNavigationMapConfig` 接收地图分辨率、刷新周期、最小命中次数、
ICP 残差、相邻位姿跳变和车身余量等参数。`initialize()` 建立本次启动的场地与原点，
`rejection_reason()` 只做安全校验，`ingest()` 记录已经被 Navigation 接受的位姿与
点云，`refresh_grid()` 按周期或强制生成新的不可变 `OccupancyGrid`。墙线共识仍在
积累时允许 ICP 点云进入可信图；墙线触发硬门限时跳过该圈点云，行为与拆分前一致。

## 相对起点坐标导航组件

文件：`components/coordinate_navigation.py`。`CoordinateNavigation` 是完整巡路运行时，
负责 D500 启动矩形标定、以启动后轴中心和启动车头重基准、可信地图更新、雷达定位
门限，以及 `Navigation` 的规划和车辆控制。HC-14、FleetBus、SSH 输入、业务 ACK 和
进程信号不在该组件内，由 `main.py` 负责接入。

组件启动后只需一次调用即可提交目标：`x/y` 单位为厘米，均相对本次启动位姿；`+X`
指向启动车头，`+Y` 在启动车头左侧；车头角度以启动车头为 `0°`，俯视逆时针为正。

```python
from components import CoordinateNavigation, CoordinateNavigationConfig

navigator = CoordinateNavigation(CoordinateNavigationConfig(allow_reverse=True))
try:
    calibration = navigator.start()  # 车辆须静止，阻塞到矩形建图完成
    navigator.navigate_to(
        x_cm=120.0,
        y_cm=80.0,
        final_heading_deg=90.0,
    )
finally:
    navigator.close()  # 停车、回中并关闭雷达
```

提交前组件会强制刷新可信栅格，并验证目标点及完整旋转车身均在拟合场地内且不与障碍
相交；未建图、已有活动任务、目标在场外或目标车身姿态不安全时会在发车前抛出
`CoordinateGoalRejected`。`cancel()` 只清当前任务，保留本次启动原点、矩形地图和累计
定位，因此后续可继续提交新坐标。

## 3×5 相邻格格心导航

文件：`components/grid_rescue_mission.py`。`GridLayout` 固定为 3 行×5 列，每格
`70 cm × 70 cm`；`AdjacentGridNavigator` 只接受上下左右相邻格，自动把目标转换为
格子中心坐标，并将车头对准本步的 `0/90/180/270°` 方向。

每步执行顺序固定为：停车、前轮回正、`InPlaceDifferentialTurn` 使用左右后轮等速反向
旋转、由 D500/Navigation 航向闭环确认角度、后轮停车，最后调用坐标导航驶向相邻格
中心。原地旋转默认速度 `80 mm/s`、航向容差 `4°`，需要两个不同雷达位姿确认；定位
超过 `0.5 s` 未更新、旋转超过 `8 s` 或收到停止请求时立即停车回中并报告失败。

该例外仅在 `disaster_rescue_main.py` 中通过
`MainConfig(allow_in_place_rotation=True)` 开启。正式 `main.py` 和普通 Navigation 默认
保持关闭，避免其他任务意外发出左右后轮反向命令。

## 正式比赛入口（Task 1 / Task 2）

当前正式比赛程序为三个文件：

- `code/main_task1.py` —— 任务1 一键入口（只负责选择任务并加载配置）；
- `code/main_task2.py` —— 任务2 一键入口（同上）；
- `code/main_radar_camera_line_following.py` —— 共享比赛核心：
  D500 矩形建图 → 起步接近 A → 沿黑线跑一圈（AB/BC/CD/DA），雷达定位为主、
  相机做有限纠偏，并上报 FleetBus 相对位置。

```bash
# 当前车默认配置（不传 --config 时自动解析）
python3 code/main_task1.py
python3 code/main_task2.py

# 其他学校：用自己的配置
python3 code/main_task1.py --config configs/my_car.toml
python3 code/main_task2.py --config configs/my_car.toml
```

任务速度、任务请求状态、完成报警时长全部来自 `[missions.task1]` /
`[missions.task2]`；比赛控制调参来自 `[missions.control]`。显式 CLI 参数始终
覆盖配置值。

串口屏启动器 `code/mission_screen_launcher.py` 监听 `MISSION1`/`MISSION2`
按钮并启动对应任务，同时把同一个 `--config` 传给子任务，保证启动器和任务
使用同一份硬件配置。systemd 通用模板见
`deploy/mission-screen-launcher.service.example`。

雷达安装偏移以后轴中心为原点，通过配置 `[sensors.radar.mount]` 或 CLI
`--radar-x-cm / --radar-y-cm / --radar-yaw-cw-deg` 提供。

### 运行日志

比赛核心每次启动都会在 `code/logs/car-main.log` 记录 DEBUG 级详细日志（本地
`code/logs/`，板端 `/home/radxa/car/logs/`），单文件 `20 MiB` 轮转、保留
`10` 个。终端日志级别由 `--log-level` 控制，目录可用 `--log-dir` 或环境变量
`CAR_LOG_DIR` 覆盖。

```bash
tail -f /home/radxa/car/logs/car-main.log
```

日志包括启动参数、矩形拟合结果、每圈雷达 ICP/墙线门限与位姿、地图刷新、
规划摘要，以及每个运动控制周期的误差/舵角/PWM/后轮/C10B 输出；不逐点转储
点云。`code/logs/` 已在 `.gitignore` 中。

### 固定赛道雷达单圈（回退入口）

`code/main_fixed_track_test.py` 是不含相机纠偏的雷达-only 回退入口；
`code/competition_task_runtime.py` 是更早的分段速度运行器。两者不是比赛正式
入口，仅用于联调/回退。旧的雷达建图/坐标导航代码归档在
`code/former_code/radar_point_navigation.py`。