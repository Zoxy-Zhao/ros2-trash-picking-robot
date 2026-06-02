# UART 串口通信协议

## 概述

NVIDIA Jetson Orin NX（上位机）与 STM32F103ZET6（下位机）之间通过 UART 串口进行双向通信。

- **物理接口**：`/dev/ttyTHS1`
- **波特率**：115200 bps
- **数据格式**：8N1（8位数据、无校验、1位停止位）
- **消息格式**：文本命令，以 `\r\n` 结尾
- **命令解析**：STM32 端 `transmit.c` 负责 DMA 接收与 FreeRTOS 消息队列分发，`freertos.c` 中各任务解析执行

## 下行指令（Jetson → STM32）

### 运动控制

| 命令 | 格式 | 说明 | 示例 |
|------|------|------|------|
| 前进 | `GO <speed>` | 以指定速度前进 | `GO 50` |
| 后退 | `BACK <speed>` | 以指定速度后退 | `BACK 30` |
| 左转 | `LEFT <angle>` | 左转指定角度 | `LEFT 90` |
| 右转 | `RIGHT <angle>` | 右转指定角度 | `RIGHT 45` |
| 停止 | `STOP` | 立即停止所有电机 | `STOP` |

### 机械臂控制

| 命令 | 格式 | 说明 | 示例 |
|------|------|------|------|
| 关节控制 | `ARM <θ1> <θ2> <θ3> <θ4>` | 设置 4 个舵机角度（整数，单位：度） | `ARM 0 150 -90 60` |
| 单舵机控制 | `SERVO <id> <angle>` | 设置单个舵机角度（id: 0-3） | `SERVO 0 90` |
| 夹爪控制 | `Claw <status>` | 0=张开，1=闭合 | `Claw 1` |

### 升降机构

| 命令 | 格式 | 说明 |
|------|------|------|
| 换桶 | `CHANGE <direction>` | 执行换桶流程 |

## 上行数据（STM32 → Jetson）

STM32 周期性上报传感器数据，包括：
- MPU6050 姿态角（yaw/pitch/roll）
- 编码器速度数据
- 超声波测距数据

## 实现细节

### STM32 端

- **接收**：UART DMA 空闲中断接收不定长数据 → FreeRTOS 消息队列 → `contact` 任务解析分发
- **解析**：`parse_command()` 函数按空格分割命令，支持最多 6 个参数
- **执行**：通过 FreeRTOS 事件组（Event Group）触发对应任务执行

### Jetson 端（ROS2）

- **发送**：`robot_serial/serial_send.py` 提供 `send_uart_message` ROS2 服务
- **调用方式**：其他节点通过 ROS2 服务客户端异步调用，指令自动加 `\r\n` 后缀发送

```python
# 调用示例（ROS2 服务客户端）
req = SendString.Request()
req.data = "ARM 0 150 -90 60"
self.uart_client.call_async(req)
```
