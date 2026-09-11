# 软件验证记录

执行日期：2026-09-11。Windows / Python 3.12.4，NumPy 1.26.4、PyYAML 6.0.1、主机 MinGW GCC 8.1.0。未连接机器人、相机或物理串口。

## 自动测试

```bash
python -m pip install -r showcase/requirements.txt
python -m unittest discover -s tests -v
python -m compileall -q robot_arm robot_core robot_vision robot_serial showcase tools
```

实际结果：**24 项全部通过，无跳过**。包括 300 组随机 FK/IK 往返、已知位姿、奇异腕、多解限位、轨迹端点及速度/加速度、连续目标确认、重复/乱序帧拒绝、共用坐标变换、两类垃圾完整抓放、独立夹爪、不可达零下发、故障锁存，以及主机编译的固件串口解析测试。

完整命令流通过 `transmit.c` 的 DMA 回调重组与 `parse_command`，每次模拟 DMA 事件输入 7 字节，C 解码结果与 Python 发送内容逐帧比较。此验证使用模拟 HAL，不执行 FreeRTOS 电机任务、不发 PWM。

## 可复现演示

```bash
python -m showcase.demo --firmware-check
python -m showcase.demo --firmware-check --fault unreachable
python -m showcase.demo --firmware-check --fault stale
python -m showcase.demo --firmware-check --fault transport
python -m showcase.demo --firmware-check --fault estop
```

| 场景 | 返回码 | 结果 | 运动样本 / UART 帧 | 虚拟时间 |
|---|---|---|---|---|
| 正常 | 0 | DONE | 139 / 142 | 17.2 s |
| unreachable | 2 | 路径预检不可达 | 0 / 0 | 0 s |
| stale | 2 | 目标未获连续新帧确认 | 0 / 0 | 0 s |
| transport | 2 | 模拟串口写失败并冻结 | 3 / 4 | 0.3 s |
| estop | 2 | 模拟急停并冻结 | 5 / 6 | 0.5 s |

正常样例使用 11 个合成新帧、dry 类别、像素 (480,435.3684)，经示例单应矩阵得到基座 (12,0,0) cm。关节采用与 ROS 配置相同的球腕模型、HOME 和桶位。

报告分别记录浮点规划角与 UART 整数指令角。正常样例最大 TCP 量化偏差约 **2.943 mm**，由两者 FK 差值计算；不是实物到位误差。虚拟时间是轨迹调度时长，不是主机运行耗时、串口时延或机器人周期实测。

每个场景生成 `output/showcase/<场景>.html/json`；默认名为 demo。报告已用无窗口 Edge 渲染检查，支持离线播放/拖动，不依赖外部网页资源。

## ROS 与硬件边界

ROS 的 `ArmControlNode` 和离线演示共用 `grasp_plan.py`；`PerspectiveNode` 和离线演示共用 `plane_geometry.py`。本机没有 rclpy，因此本机未运行 ROS 2 服务集成测试。仓库 CI 保留 ROS 2 Humble 容器构建和 `tools/check_ros_integration.py`，其实际运行状态以 GitHub Actions 为准。

目前不验证机械碰撞、舵机跟踪、真实 UART 到位反馈、物理急停、相机标定精度或果实/垃圾实际抓取。路径点提升至相同高度并不保证关节插值的整条 TCP 路径保持恒高。模拟接收检查模型限位，不等价于 STM32 的舵机 PWM 零位/行程校准。

YOLO/TensorRT 未在本机演示中运行；性能工具需在实际 Jetson 与模型环境执行。仓库没有新增 FPS、抓取成功率或自主换桶完整闭环成绩。
