import rclpy
from interfaces.srv import ArmControl, Boolean, Perspective

class Client:
    def __init__(self, node):
        self.node = node
        self.arm_control = self.node.create_client(ArmControl, "arm_control")
        self.enable_detection = self.node.create_client(Boolean, "enable_detection")
        self.perspective_server = self.node.create_client(Perspective, "perspective_service")

    def fetch_object(self, x, y, z):

        while not self.arm_control.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('等待服务启动...')

        request = ArmControl.Request()
        request.type = "fetch"
        request.x = x
        request.y = y + 2.3
        request.z = 0.0
        request.angle = 60.0
        future = self.arm_control.call_async(request)
        self.node.get_logger().info('已发送机械臂控制信息')
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

    def stop_yolo(self):
        # 使yolo服务暂停
        while not self.enable_detection.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("等待服务器响应")

        request = Boolean.Request()
        request.status = False
        self.enable_detection.call_async(request)
        self.node.get_logger().info('yolo服务已暂停')

    def start_yolo(self):
        # 使yolo服务暂停
        while not self.enable_detection.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("等待服务器响应")

        request = Boolean.Request()
        request.status = True
        self.enable_detection.call_async(request)
        self.node.get_logger().info('yolo服务已开启')

    def get_perspective(self, x, y):

        while not self.perspective_server.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("等待服务器响应")
        request = Perspective.Request()
        request.x = float(x)
        request.y = float(y)
        future = self.perspective_server.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if not future.done():
            self.node.get_logger().error("服务响应超时")
            return None, None
        
        response = future.result()
        if response.success:
            self.node.get_logger().info(f"服务调用成功，返回 x: {response.x_real}, y: {response.y_real}")
            return response.x_real, response.y_real
        else:
            self.node.get_logger().error("服务调用失败")
            return None, None
