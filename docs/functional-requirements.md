# 功能模块与检查方法

| 能力 | 实现入口 | 检查方法 |
|---|---|---|
| 六轴几何 FK/IK、多解筛选、限位与可达性 | `robot_arm/robot_arm/six_axis.py` | 已知位姿、随机位姿往返、奇异构型与限位测试 |
| 关节插值与分类抓取路径 | `move_joints.py`、`arm_control.py` | 轨迹端点、速度、加速度与路径点 IK 检查 |
| ROS 2 异步任务调度 | `robot_core/robot_core/confirm.py` | 状态流程检查；机械臂服务提供 ROS 集成测试 |
| 连续新帧与目标一致性确认 | `target_gate.py`、`yolo_detection.py` | 连续帧、空帧、类别变化与位置跳变测试 |
| 标定平面 X/Y 定位 | `perspective.py` | 标定矩阵校验与坐标映射 |
| YOLOv5 TensorRT 导出与加载 | `inference.py`、`tools/export_tensorrt.py` | 根据模型系列导出并加载本地引擎 |
| 检测性能统计 | `tools/benchmark_detection.py` | 视频逐帧计时，生成 FPS、延迟分位数与 CSV |
| UART 六关节帧与下位机解析 | `serial_send.py`、`transmit.c` | GCC 编译、分包、连包与非法输入测试 |
| 六舵机驱动与夹爪分离 | `freertos.c`、`arm6_config.h` | 通道映射检查与舵机参数配置 |
| Web MQTT 控制与状态展示 | `web/index.html` | 状态订阅与命令发布；机器人端桥接需接入部署系统 |
| 换桶机构 | `firmware/Core/Src/lift.c` | 固件升降换桶动作；导航与满溢感知作为外部输入 |

## 自动化检查

GitHub Actions 包含算法与串口测试、Python 语法检查，以及 ROS 2 Humble 包构建和机械臂服务集成测试。运行记录见仓库 Actions 页面。

六轴模式的构型、坐标、参数和路径约定见 `system-design.md`；硬件连接、舵机配置与软件启动见 `deployment-guide.md`。
