# 图像发布节点
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image', 10)
        self.timer = self.create_timer(0.033, self.publish_frame)  # 以 ~30 FPS 的速度发布帧
        self.bridge = CvBridge()
        self.video_capture = cv2.VideoCapture(self.gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

        if not self.video_capture.isOpened():
            self.get_logger().error("无法打开摄像头。")
            raise RuntimeError("Camera unavailable")

        self.get_logger().info("摄像头已成功打开。")

    def gstreamer_pipeline(
        self,
        sensor_id=0,
        capture_width=1920,
        capture_height=1080,
        display_width=960,
        display_height=540,
        framerate=30,
        flip_method=0,
    ):
        return (
            "nvarguscamerasrc sensor-id=%d ! "
            "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink drop=true max-buffers=1"
            % (
                sensor_id,
                capture_width,
                capture_height,
                framerate,
                flip_method,
                display_width,
                display_height,
            )
        )

    def publish_frame(self):
        ret_val, frame = self.video_capture.read()
        if ret_val:
            # 将 OpenCV 帧转换为 ROS 图像消息
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "camera"
            self.publisher_.publish(ros_image)
        else:
            self.get_logger().error("从摄像头读取帧时出错。")

    def __del__(self):
        if self.video_capture.isOpened():
            self.video_capture.release()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)

    # 清理资源
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
