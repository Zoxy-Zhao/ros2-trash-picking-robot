# 当前能力与简历对应

| 简历能力 | 对应代码 | 可演示或验证的内容 |
|---|---|---|
| ROS 2 系统集成与异步调度 | robot_core/robot_core/confirm.py、robot_arm/robot_arm/arm_control.py | 检测暂停、定位、抓取、恢复；服务超时锁存 |
| 六轴几何 FK/IK 与可达性（软件验证） | robot_arm/robot_arm/six_axis.py | 300 个随机位姿往返、奇异腕、多解、限位与种子选解 |
| 视觉定位与目标确认 | robot_core/robot_core/target_gate.py、robot_vision/robot_vision/plane_geometry.py | 类别/位置一致性、时间戳去重、标定平面到基座坐标 |
| 分类抓放路径 | robot_arm/robot_arm/grasp_plan.py | 七个执行阶段，共用 ROS/离线入口，全部点先求解 |
| 关节轨迹与 UART | robot_arm/robot_arm/move_joints.py、robot_serial/robot_serial/serial_send.py | 同步五次插值、整数 ARM6 编码、独立 CLAW |
| 固件指令解析验证 | firmware/Core/Src/transmit.c、showcase/firmware_receiver.c | 实际 C 解析器，模拟 HAL 的分片命令流验证 |
| TensorRT 部署与测量 | tools/export_tensorrt.py、tools/benchmark_detection.py | FP16 引擎导出、加载、逐帧时延和吞吐统计；需目标设备实跑 |
| 当前能力展示 | showcase/demo.py、showcase/report.html | 离线轨迹回放、六轴角度、夹爪、状态和指令量化误差 |

推荐简历将六轴链路表述为“软件验证”或“模拟执行验证”，将 TensorRT 表述为实际存在的导出/加载/测量能力。当前没有随仓库提供稳定 30 FPS 的实测报告，不使用该数字代替测量。

软件模型默认尺寸和标定样例不代表具体安装机构。几何 IK 浮点精度、整数指令误差与物理到位误差是三种不同指标。详细结果见 [软件验证记录](software-validation.md)。
