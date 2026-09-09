# 串口发送节点
import rclpy
from rclpy.node import Node
from interfaces.srv import SendString  # 使用自定义服务
import serial
import time
class UARTSenderService(Node):
    def __init__(self):
        super().__init__('uart_sender_service')
        self.srv = self.create_service(
            SendString, 
            'send_uart_message', 
            self.send_message_callback
        )
        
        self.declare_parameter("port", "/dev/ttyTHS1")
        self.serial_port = serial.Serial(
            port=self.get_parameter("port").value,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1, write_timeout=1.0
        )
        time.sleep(1)  # 等待串口初始化
        self.get_logger().info("UART发送服务已就绪")

    def send_message_callback(self, request, response):
        try:
            payload = request.data.strip()
            if not payload or "\n" in payload or "\r" in payload or len(payload.encode("ascii")) > 62:
                raise ValueError("Invalid UART command frame")
            message = payload + "\r\n"
            if self.serial_port.write(message.encode("ascii")) != len(message):
                raise IOError("Incomplete UART write")
            self.serial_port.flush()  # 强制刷新缓冲区
            self.get_logger().info(f"已发送: {message.strip()}")
            response.success = True
            response.message = f"消息 '{request.data}' 发送成功"
        except Exception as e:
            self.get_logger().error(f"发送消息出错: {str(e)}")
            response.success = False
            response.message = f"发送失败: {str(e)}"
        return response
    
def main(args=None):
    rclpy.init(args=args)
    uart_sender_service = UARTSenderService()
    try:
        rclpy.spin(uart_sender_service)
    except KeyboardInterrupt:
        pass
    finally:
        uart_sender_service.serial_port.close()
        uart_sender_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()