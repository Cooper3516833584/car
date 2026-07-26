# Grid Coordinate Main - Turn-and-Drive Navigation

## 概述

`grid_coordinate_main.py` 实现了"原地转向 + 直线行驶"的坐标导航模式，每次接收目标坐标后：

1. **原地转向**：计算当前位置到目标的方位角，使用 `InPlaceDifferentialTurn` 原地旋转到该方向
2. **直线行驶**：调用现有的 `CoordinateNavigation` 前往目标
3. **自动修正**：到达目标附近如有残差超过阈值，自动重复"转向→行驶"最多 3 次

## 核心特性

- **显式原地旋转**：每段行驶前先停车回中前轮，再原地差速旋转到目标方位
- **残差修正**：到达目标附近（< 25cm）但残差 > 15cm 时自动启动修正迭代
- **可选最终航向**：支持第三参数指定最终朝向，仅在接近目标（< 25cm）时应用
- **安全限制**：同一时间只允许一个任务，超出场地边界的目标会被拒绝
- **现有导航**：直行阶段复用现有 Navigation 的路径规划、避障和雷达闭环反馈

## 使用方法

### 启动

```bash
# 板端启动（与 main.py 参数相同）
sudo python3 /home/radxa/car/grid_coordinate_main.py \
  --radar-x-cm 0 --radar-y-cm 0 --radar-yaw-cw-deg 0 \
  --allow-reverse
```

### 控制台命令

启动后等待矩形场地标定完成，终端显示：

```
Grid Coordinate Navigation Ready
Field bounds: x=[...,...] y=[...,...] cm

Commands:
  <x_cm> <y_cm> [heading_deg]  - Navigate with turn-and-drive
  status                        - Show current state
  stop                          - Cancel active navigation
  help                          - Show this message
  quit                          - Exit application
```

**示例**：

```
> 100 50          # 前往 (100, 50)，无最终航向约束
> 200 100 90      # 前往 (200, 100)，最终朝向 90°（左转）
> status          # 查看当前位姿和状态
> stop            # 取消当前任务
> quit            # 退出程序
```

## 参数约定

- **坐标系**：与 `main.py` 一致，启动后轴中心为 `(0, 0)`，启动车头为 `0°`
- **航向角**：`0..359` 整数，俯视逆时针为正（`0°` 前、`90°` 左、`180°` 后、`270°` 右）
- **容差**：位置 `10 cm`，航向 `8°`

## 配置常量

位于 `grid_coordinate_main.py` 顶部：

```python
MAX_CORRECTION_ITERATIONS = 3      # 最大修正次数
CORRECTION_THRESHOLD_CM = 15.0     # 触发修正的残差阈值
FINAL_TOLERANCE_CM = 10.0          # 最终到达容差
APPROACH_TOLERANCE_CM = 25.0       # 应用最终航向的距离阈值
STEP_TIMEOUT_S = 90.0              # 单段导航超时
```

## 与 main.py 的区别

| 特性 | main.py | grid_coordinate_main.py |
|------|---------|------------------------|
| 原地旋转 | 不使用 | 每段前显式转向 |
| 路径规划 | Hybrid A* + Dubins | 现有导航（允许微调） |
| 残差处理 | 一次到达 | 自动修正迭代 |
| 适用场景 | 通用阿克曼导航 | 格子式定点任务 |

## 实现文件

- **主程序**：`code/grid_coordinate_main.py`
- **测试**（简化版）：`code/test/test_grid_coordinate_main_simplified.py`
- **依赖组件**：
  - `components/grid_rescue_mission.py` 的 `InPlaceDifferentialTurn`
  - `components/coordinate_navigation.py` 的 `CoordinateNavigation`
  - `main.py` 的 `CarMainApplication` 基类

## 注意事项

1. **必须启用原地旋转**：`MainConfig(allow_in_place_rotation=True)` 已在代码中强制要求
2. **前轮必须回中**：原地旋转前自动停车并回正前轮，避免轮胎磨损
3. **雷达航向闭环**：原地旋转使用雷达位姿闭环，需要 2 个不同样本确认到角度
4. **不会隐式扩散**：原地旋转能力仅在此入口启用，不影响 `main.py` 的默认行为

## 部署

```bash
# 本地同步到板端（已在上一会话中完成）
# 板端测试
ssh radxa@192.168.31.224
cd /home/radxa/car
sudo python3 grid_coordinate_main.py --help
```

2026-07-26 实现完成。
