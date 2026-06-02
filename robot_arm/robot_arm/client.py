# 发送节点（集成逆运动学服务调用）
import rclpy
from rclpy.node import Node
from interfaces.srv import ArmControl

class MoveJointsClient(Node):
    def __init__(self):
        super().__init__('move_joints_client')
        
        self.client = self.create_client(ArmControl, "arm_control")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务启动...')

    def send_request(self):
        request = ArmControl.Request()
        request.type = "fetch"
        request.x = 20.0
        request.y = 0 + 1.5
        request.z = 0.0
        # request.angle = 50.0
        self.client.call_async(request)
        self.get_logger().info('已发送')

def main(args=None):
    rclpy.init(args=args)
    node = MoveJointsClient()
    node.send_request()
    rclpy.shutdown()

if __name__ == '__main__':
    main()