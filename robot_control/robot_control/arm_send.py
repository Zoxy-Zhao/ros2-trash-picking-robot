import rclpy
from rclpy.node import Node
from interfaces.srv import InverseKinematics  # 假设这是逆运动学的服务接口
from interfaces.srv import SendString  # 自定义服务接口，用于发送字符串消息
import time

class IKClient(Node):
    def __init__(self):
        super().__init__('ik_client')
        # 创建客户端，用于请求逆运动学计算服务
        self.ik_cli = self.create_client(InverseKinematics, 'calculate_ik')
        
        # 等待逆运动学服务可用
        while not self.ik_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待逆运动学服务就绪...')

        # 创建客户端，用于发送串口消息
        self.uart_cli = self.create_client(SendString, 'send_uart_message')
        
        # 等待串口发送服务可用
        while not self.uart_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待串口发送服务就绪...')

    def send_request(self, x, y, z, angle):
        req = InverseKinematics.Request()
        req.x = x
        req.y = y
        req.z = z
        req.angle = angle
        
        future = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            return future.result()
        else:
            raise RuntimeError('逆运动学服务调用失败')

    def send_uart_message(self, message):
        req = SendString.Request()
        req.data = message
        
        future = self.uart_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info(f"成功发送: {message}")
        else:
            self.get_logger().error("发送串口消息失败")

def main():
    rclpy.init()
    client = IKClient()

    try:
        response = client.send_request(
            x=20.0,
            y=0.0 + 2.75,
            z=0.0,
            angle=60.0
        )
        
        if response.success:
            angles = [
                int(round(response.theta1)),
                int(round(response.theta2)),
                int(round(response.theta3)),
                int(round(response.theta4))
            ]
            
            for i, angle in enumerate(angles):
                message = f"SERVO {i} {angle}"
                client.send_uart_message(message)
                client.get_logger().info(f"发送: {message}")
                time.sleep(0.1)  # 简单的延时以确保消息被单独处理
            
            print(f"计算成功！角度（已四舍五入）："
                  f"\nθ1 = {angles[0]}°"
                  f"\nθ2 = {angles[1]}°"
                  f"\nθ3 = {angles[2]}°"
                  f"\nθ4 = {angles[3]}°")
            
        else:
            print(f"错误：{response.error_message}")
            
    except Exception as e:
        client.get_logger().error(f'发生异常: {e}')
        
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()