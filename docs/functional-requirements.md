# 功能范围与验证状态

| 能力 | 实现入口 | 当前验证范围 |
|---|---|---|
| 六轴几何 FK/IK、多解筛选、限位与可达性 | `robot_arm/robot_arm/six_axis.py` | 参考构型的软件测试；实际六轴硬件参数待标定 |
| 关节插值与分类抓取路径 | `move_joints.py`、`arm_control.py` | 轨迹数值测试、参考路径 IK 检查；无真机抓取结果 |
| ROS 2 异步任务调度 | `robot_core/robot_core/confirm.py` | 提供 ROS 集成检查；以 CI/目标机器执行结果为准 |
| 连续新帧与目标一致性确认 | `target_gate.py`、`yolo_detection.py` | 确认策略单元测试；推理需模型与相机 |
| 标定平面 X/Y 定位 | `perspective.py` | 代码与输入校验；实际标定矩阵未随仓库提供 |
| YOLOv5 TensorRT 导出与加载 | `inference.py`、`tools/export_tensorrt.py` | 实现工具与加载接口；无模型、无 Jetson 实测 |
| 30 FPS 检测目标 | `tools/benchmark_detection.py` | 待测，不作为已达成指标 |
| UART 六关节帧与下位机解析 | `serial_send.py`、`transmit.c` | 主机 GCC 编译与分包/非法输入测试；非完整固件构建 |
| 六舵机驱动与夹爪分离 | `freertos.c`、`arm6_config.h` | 通道映射检查；完整固件烧录和六轴真机待验证 |
| Web MQTT 控制与状态展示 | `web/index.html` | 保留现有实现；缺完整机器人端桥接 |
| 换桶机构 | `firmware/Core/Src/lift.c` | 保留原固件动作；自主到站、满溢检测与完整闭环待集成 |

原型照片与 CAD 不代表六轴版本已经制造。软件更新不补写历史竞赛中的未验证结果。模型、标定与性能报告应以真实运行产物补充。
