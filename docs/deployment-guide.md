# 部署指南

## 1. 构建环境

ROS 2 Humble 应使用匹配的运行环境。Jetson 的 Ubuntu/JetPack 组合需先确认兼容；若设备系统不匹配，应选择经验证的容器方案，并验证相机、串口与 GPU 映射。不要直接把 Ubuntu 20.04 与 Humble 当作默认可用组合。

ROS 依赖在各包 `package.xml` 中声明。模型推理所需 PyTorch、TensorRT 和 YOLOv5 请按目标 JetPack 与训练版本安装，不要用通用桌面 PyTorch 包替换 Jetson 的兼容构建。OpenCV 需支持实际使用的 GStreamer 相机管线。

```bash
cd ~/robot_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

`ArmControl.srv` 增加 `roll/pitch/yaw/class_name`，请重新构建所有调用方；旧 `angle` 字段保留但忽略。

## 2. 无硬件计算模式

```bash
ros2 launch robot_launch arm.py
ros2 service call /arm_control interfaces/srv/ArmControl "{type: fetch, x: 12.0, y: 0.0, z: 0.0, roll: 0.0, pitch: 180.0, yaw: 0.0, class_name: dry}"
```

在两个已加载工作空间的终端分别执行。默认六轴参考模型、`dry_run: true`、不开串口。服务完成意味着路径与命令生成成功，不意味着真机抓取。可执行 `python tools/check_ros_integration.py` 检查真实 ROS 服务调用及拒绝不可达目标的行为。

## 3. 配置实际六轴硬件

复制 `robot_launch/config/robot.yaml` 到自己的配置路径，按 `system-design.md` 的轴线约定测量连杆和工具长度，填入真实关节限位、待机角和分类桶坐标。若实际机械臂不是 Z-Y-Y + Z-Y-Z 球形腕构型，应先修改运动学模型。

在 `firmware/Core/Inc/arm6_config.h` 中校准每关节 PWM 通道、方向、零位和行程，使上位机限位与下位机可执行范围一致；通道 5 留给夹爪。参考配置不是原型实测值。完成校准后将 `ARM6_ENABLED` 设为 1，使用 STM32CubeMX / CubeIDE 生成缺失 HAL 工程资源并编译烧录。

本次软件检查只编译了串口模块的主机测试，未完成完整 STM32 工程编译或烧录。六轴固件启动姿态使用 `ARM6_HOME_DEG`，须同步校准并确认它与 ROS 配置 `home_deg` 一致；四轴模式保留原型初始化。当前没有自动回零或编码器闭环。

完成上述工作后，在个人配置中设置 `hardware_calibrated: true`、`dry_run: false`，再开启串口：

```bash
ros2 launch robot_launch arm.py config:=/absolute/path/robot.yaml serial:=true
```

原四轴样机使用 `arm_dof: 4` 和原有固件映射，勿直接使用六轴参数。四轴模式保留历史 IK，不能约束完整末端姿态。

## 4. 视觉与坐标

1. 用实际相机安装、分辨率和抓取平面做单应性标定，将矩阵路径填入 `matrix_path`。
2. 核对 X/Y 为基座坐标且单位 cm，设置 `grasp_z_cm` 与 `grasp_rpy_deg`。
3. 将模型类别映射到 `dry_classes` 和 `wet_classes`，填写实际桶坐标。
4. 按 [TensorRT 指南](tensorrt-deployment.md)导出并实测模型，配置 `model_path`、`model_family` 和 `yolov5_repo`。
5. 先以计算模式启动全部节点，再用硬件配置联调。

```bash
ros2 launch robot_launch start.py config:=/absolute/path/robot.yaml
```

所有节点使用同一已配置的 Python/ROS 环境，启动文件不再拼接虚拟环境 shell 命令。

## 5. 状态与故障

服务不可用不会触发抓取。不可达路径、未知类别会返回失败；发送超时或执行状态未知会锁定故障，需要检查机器状态并恢复待机位置后重启。没有 MCU ACK、碰撞检测或机械臂急停服务，不能把软件等待完成当作物理到位。

Web 页面保留 MQTT 交互能力；完整机器人端状态发布、指令桥接和自主换桶导航未包含在本次扩展中。
