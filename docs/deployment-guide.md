# 部署指南

## 1. 硬件连接

### Jetson Orin NX

| 接口 | 连接 |
|------|------|
| CSI-0 | 全局摄像头（GStreamer `nvarguscamerasrc sensor-id=0`） |
| UART (`/dev/ttyTHS1`) | STM32 UART（TX↔RX 交叉连接） |
| USB / Display | 调试用显示器、键盘 |

### STM32F103ZET6

| 外设 | 引脚/接口 | 说明 |
|------|-----------|------|
| 电机 A（左前） | TIM1 PWM + GPIO | 编码器 TIM2 |
| 电机 B（右前） | TIM1 PWM + GPIO | 编码器 TIM3 |
| 电机 C（左后） | TIM1 PWM + GPIO | 编码器 TIM4 |
| 电机 D（右后） | TIM1 PWM + GPIO | 编码器 TIM5 |
| PCA9685（舵机驱动） | I2C | 4路机械臂舵机 + 升降机构舵机 |
| MPU6050 | I2C | 姿态检测 |
| OLED (SSD1306) | I2C | 128x64 状态显示 |
| 超声波 HC-SR04 | GPIO（Trig + Echo） | 距离检测 |
| UART6 | DMA 收发 | 与 Jetson 通信 |

## 2. Jetson 环境配置

### 系统要求

- Ubuntu 20.04+ (JetPack)
- ROS2 Humble
- CUDA (用于 YOLOv5 GPU 加速)
- Python 3.10+

### 安装步骤

```bash
# 1. 安装 ROS2 Humble（参考官方文档）
# https://docs.ros.org/en/humble/Installation.html

# 2. 创建 Python 虚拟环境（YOLO 等依赖需要隔离安装）
python3 -m venv ~/robot_venv
source ~/robot_venv/bin/activate
pip install ultralytics opencv-python pyserial numpy torch

# 3. 创建 ROS2 工作空间
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# 4. 将项目源码放入 src/
# 确保 interfaces, robot_arm, robot_control, robot_core,
# robot_launch, robot_serial, robot_vision 都在 src/ 下

# 5. 编译
cd ~/robot_ws
colcon build
source install/setup.bash

# 6. 放置模型权重
# 将 YOLOv5 训练好的 best.pt 放到 robot_vision/model/ 目录
# 将相机标定矩阵 matrix.npy 放到 robot_vision/data/ 目录
```

### 运行

```bash
# 一键启动所有节点
ros2 launch robot_launch start.py

# 或单独启动各模块
ros2 launch robot_launch arm.py    # 仅机械臂
ros2 launch robot_launch yolo.py   # 仅视觉检测
```

## 3. STM32 固件烧录

1. 使用 **STM32CubeIDE** 打开 `firmware/freertos_chuankou.ioc`
2. 通过 CubeMX 生成 HAL 驱动代码
3. 编译项目
4. 通过 ST-Link 烧录到 STM32F103ZET6

## 4. Web 控制面板

Web 控制面板为独立的前端页面，直接在浏览器中打开 `web/index.html` 即可。

完整的 MQTT 通信对接需要：
1. 部署 EMQX 服务器（推荐 Docker 方式）
2. 配置 Jetson 端 MQTT 客户端发布状态数据
3. Web 端通过 WebSocket 连接 EMQX 订阅数据

```bash
# Docker 部署 EMQX
docker run -d --name emqx \
  -p 1883:1883 \
  -p 8083:8083 \
  -p 18083:18083 \
  emqx/emqx:5.7.2
```

## 5. 注意事项

- 代码中部分路径为硬编码（如 `/home/orin/WorkSpace/...`），部署到新环境时需要修改对应的文件路径
- YOLOv5 模型权重文件（`.pt`）未包含在 Git 仓库中，需自行训练或获取
- 相机标定矩阵（`matrix.npy`）与具体摄像头和安装位置相关，更换硬件后需重新标定（运行 `perspective.py` 的标定功能）
