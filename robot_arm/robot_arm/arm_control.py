import rclpy
from rclpy.node import Node
from interfaces.srv import ArmControl, SendString
from robot_arm.move_joints import MoveJoints
from robot_arm.ik_service import IKService
import time

SPEED = 5

class ArmControlNode(Node):
    def __init__(self):
        super().__init__('arm_control')

        self.move_joints = MoveJoints(self)
        self.ik_server = IKService()

        self.uart_client = self.create_client(SendString, 'send_uart_message')

        self.theta_now = [
            0.0,
            150.0,
            -90.0,
            60.0
        ]
        
        # 等待服务启动
        while not self.uart_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 UART 服务就绪...')

        self.server = self.create_service(ArmControl, "arm_control", self.arm_control_server)

    def arm_control_server(self, request, response):
        match request.type:
            case "fetch":
                self.fetch_object(request.x, request.y, request.z)
        response.success = True  
        self.get_logger().info("机械臂运动完成")
        return response

    def fetch_object(self, x, y, z):
        
        theta_target = self.ik_server.ik_solution(x, y, z)
        self.get_logger().info("逆运动学解算完毕")

        self.claw_status(0)

        self.move_joints.move_joints(self.theta_now, theta_target, SPEED)

        time.sleep(1)
        self.claw_status(1)
        time.sleep(1)

        self.move_joints.move_joints(theta_target, self.theta_now, SPEED)

        time.sleep(1)

        self.claw_status(0)


    def uart_send(self, string):
        req = SendString.Request()
        req.data = string
        self.uart_client.call_async(req) 

    def claw_status(self, status):
        if status:
            self.uart_send("Claw 1")
        else:
            self.uart_send("Claw 0")

def main(args=None):
    rclpy.init(args=args)
    node = ArmControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()