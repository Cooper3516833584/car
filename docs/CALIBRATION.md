# 实车标定手册（CALIBRATION.md）

按以下顺序操作。每一项完成后把测量结果填进配置文件（复制
`configs/car.example.toml` 得到自己学校的配置）。**不要**修改主程序。

> 单位注意：TOML 里长度用 mm、速度用 cm/s、角度用 rad（舵机）/ deg（雷达安装）。

## 0. 准备工作

```bash
cp configs/car.example.toml configs/my_car.toml
export CAR_CONFIG=configs/my_car.toml
```

## 1. 测轴距（wheelbase_mm）

把车放平，用卷尺/卡尺量**前后轴中心**的距离（不是车身长度）。

```toml
[vehicle.geometry]
wheelbase_mm = 142.5
```

## 2. 测左右后轮中心距（physical_track_width_mm）

量左右后轮**中心**的距离。通常先量车轮厚度和左右外侧总宽再相减：

```toml
wheel_thickness_mm = 26.4
outer_wheel_width_mm = 143.5
# physical_track_width_mm = outer_wheel_width_mm - wheel_thickness_mm = 117.1
physical_track_width_mm = 117.1
```

## 3. 测车身长宽（body_length_mm / body_width_mm）

碰撞检测用矩形车身尺寸：

```toml
body_length_mm = 230.0
body_width_mm = 145.0
```

## 4. 测后轴到车身中心（rear_axle_to_body_center_mm）

导航位姿原点在后轴中心。车身前后悬对称时 = `wheelbase / 2`（当前车
`71.25 mm`）。若前后悬不对称，实测后轴中心到车身几何中心的纵向距离。

## 5. 舵机回中（center_us）

把舵机接到目标 PWM 输出，**断电状态下**手动把前轮摆到目视正前方，记录
当前 PWM 脉宽；或先用厂家默认（1500 us）上电，再微调脉宽让前轮完全居中。
当前车实测 `1580 us`。

```toml
[vehicle.steering]
center_us = 1580
```

## 6. 确认转向方向（direction_sign）

- 正角度（逻辑左转）输出脉宽应 **> center_us**，且前轮实际向左偏；
- 负角度（逻辑右转）输出脉宽应 **< center_us**，且前轮实际向右偏。

若方向相反，`direction_sign` 取反（当前车实测 `-1.0`）。

> ⚠️ 标定期间小车必须架起，后轮悬空，防止意外移动。

## 7. 标定左右机械极限（logical_right_max_rad / logical_left_max_rad）

缓慢增大转角直到舵机/连杆到达机械限位前，记录能稳定到达的最大正负角度。

```toml
logical_right_max_rad = -0.32
logical_left_max_rad = 0.49
```

（`calibration_min_rad` / `calibration_max_rad` 是厂家曲线的角度范围，
一般沿用厂家数据 `-0.49 / +0.32`。）

## 8. 标定角度→PWM（曲线系数）

若沿用 WHEELTEC L150 厂家三次曲线，填写：

```toml
curve_a3 = -0.628
curve_a2 = 1.269
curve_a1 = -1.772
curve_a0 = 1.573
curve_scale = 640.62
factory_center_us = 1501
```

换非 L150 舵机/转向机构时，需要重新拟合角度→脉宽映射，并把系数填到这里。

**回归检查**（当前车）：`-0.12 rad → 1454 us`，`0 → 1580 us`，
`+0.12 rad → 1728 us`。可用 `python3 -m pytest code/test/test_steering_config.py`
验证。

## 9. 测雷达相对后轴位置（sensors.radar.mount）

以**后轴中心**为原点：

```toml
[sensors.radar.mount]
x_forward_cm = 0.0   # 雷达测量原点在后轴前方 cm（在后方填负数）
y_left_cm = 0.0      # 在后轴左侧 cm（右侧填负数）
yaw_cw_deg = 0.0     # 雷达零度相对车头顺时针偏角
```

## 10. 标定摄像头透视（sensors.camera.perspective）

把车停在场地上，前放一张 A4 纸或标记物：

1. 在俯视图像上标出近端左右、远端左右四个角点；
2. 记录四点在**归一化坐标**（除以宽高）中的位置，按 左下、右下、右上、左上
   顺序填 `source_points_norm`；
3. 量出梯形对应的地面宽/深（cm）填 `ground_width_cm` / `ground_depth_cm`。

同时按当前场地黑线实测填写 `[sensors.camera.line]`（线宽窗口、扫描距离、
形态学尺寸等）。

## 11. 低速测试

```bash
python3 code/main_task1.py --config configs/my_car.toml --no-camera-correction \
  --ab-speed-cm-s 4 --bc-speed-cm-s 4 --cd-speed-cm-s 4 --da-speed-cm-s 4
```

- 先确认 C10B 电机已使能（串口遥测第 2 字节为 `00`）；
- 确认 D500 建图成功、车能沿黑线低速走完一圈；
- 确认声光报警 on/off 正常（`active_low` 是否正确）。

## 12. 比赛参数调优

把 `[missions.task1]` / `[missions.task2]` 的速度调到比赛值，再按
`[missions.control]` 逐项调相机纠偏增益/死区/各段进度窗口。每次只改一个
参数，记录效果。
