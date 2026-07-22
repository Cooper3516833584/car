# 测试与临时工具

此目录存放硬件联调脚本、一次性配置工具、固件试验补丁及单元测试。它们不是最终上位机主程序的一部分。

- `wheel_test.py`：后轮低速实车测试。
- `steering_servo_test.py`：前轮转向舵机测试。
- `servo_pulse_test.py`：直接修改文件顶部 `PULSE_US` 后点击运行，仅输出前轮舵机原始
  PWM，不访问后轮电机。
- `drive_forward_1m_test.py`：点击运行后仅控制后轮，以默认 `1 m/s` 开环前进约 `1 m`
  并自动停车；保持当前舵机 PWM 不变，需先运行 `servo_pulse_test.py` 设置直行方向。
- `hc14_*.py`：HC-14 查询、配置和双向链路测试。
- `d500_uart_probe.py`：只读打开 `/dev/ttyS6`，统计 D500 `54 2C` 有效包、CRC 和完整
  圆周；不写雷达、不访问驱动板、舵机或电机。
- `gimbal_*`、`c10b_a2_a3_gimbal.patch`：已停止采用的云台临时方案。
- `test_rear_motor.py`：后驱组件的纯软件协议与运动学单元测试，不访问真实串口。
- `test_ackermann_drive.py`：转向标定、偏航方向及前后轮联动的纯软件单元测试，不访问真实 PWM 或串口。
- `test_serial_communication.py`：HC-14桥封装、分片重组、噪声重同步及组件参数验证，不访问真实串口。
- `test_radar_driver.py`：D500 分段收包、CRC 错帧重同步、完整圆周拼接、雷达安装
  旋转、无人机全局参考变换、顺时针正角 ICP、矩形墙线绝对观测、异常残差拒绝及
  ICP 状态纠漂回写、启动矩形场地拟合及零点/零度角建立，不打开真实串口。
- `test_navigation.py`：导航角度转换、实车矩形碰撞、Hybrid A*、可选最终朝向、
  倒车开关、Pure Pursuit、换向前停车、定位超时和到达停车，不访问真实 PWM 或串口。
- `test_navigation_protocol.py`：坐标/可选航向编解码、V2 HMAC、防篡改、命令去重和
  停止命令测试，不访问真实 HC-14。
- `test_main.py`：正式主程序的启动 `(0,0,0°)` 坐标重基准、旋转矩形外禁行、SSH
  `x y [heading]` 解析、角度范围、坐标转交及越界拒绝测试，不启动雷达、串口、PWM
  或电机。

运行单元测试：

```bash
python3 -m unittest discover -s code/test -p 'test_*.py'
```
