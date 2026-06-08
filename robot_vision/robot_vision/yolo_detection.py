# YOLO识别节点
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from interfaces.msg import BoundingBox, BoundingBoxArray
from interfaces.srv import Boolean
from ultralytics import YOLO
import torch
import cv2
import threading
import time


class YOLODetector(Node):
    def __init__(self):
        super().__init__("yolo_detector")
        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_lock = threading.Lock()

        # 加载YOLO模型
        torch.backends.cudnn.enabled = True
        torch.backends.cudnn.benchmark = True
        # 模型路径可通过 ROS2 参数 model_path 覆盖，默认指向工作空间下的权重文件
        self.declare_parameter(
            "model_path",
            os.path.expanduser("~/robot_ws/src/robot_vision/model/best.pt"),
        )
        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.model = YOLO(model_path)
        self.model.fuse()

        # 订阅摄像头
        self.subscription = self.create_subscription(
            Image,
            "camera/image",
            self.image_callback,
            qos_profile=rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value,
        )

        # 服务
        self.enable = True
        self.server = self.create_service(
            Boolean, "enable_detection", self.enable_detection
        )

        # 发布者
        self.bbox_pub = self.create_publisher(BoundingBoxArray, "detection_boxes", 10)

        # 启动处理线程
        self.process_thread = threading.Thread(target=self.process_frame)
        self.process_thread.start()

        self.get_logger().info(f"YOLO节点已启动，CUDA可用: {torch.cuda.is_available()}")

    def enable_detection(self, request, response):
        self.enable = request.status
        response.success = True
        return response

    def image_callback(self, msg):
        """异步接收图像帧，仅更新最新帧"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.frame_lock:
                self.latest_frame = cv_image
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")

    def process_frame(self):
        """专用处理线程"""
        while rclpy.ok():
            if self.enable is not True:
                time.sleep(0.01)
                continue

            start_time = time.time()

            # 获取当前帧
            with self.frame_lock:
                if self.latest_frame is None:
                    time.sleep(0.001)
                    continue
                current_frame = self.latest_frame.copy()

            # YOLO推理
            results = self.model(current_frame, imgsz=640, verbose=False)

            # 创建结果消息
            bbox_array = BoundingBoxArray()
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "camera"
            bbox_array.header = header

            # 解析检测结果
            if results[0].boxes is not None:
                for box in results[0].boxes:
                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                    conf = box.conf[0].cpu().numpy().item()
                    cls_id = int(box.cls[0].cpu().numpy().item())

                    bbox = BoundingBox()
                    bbox.class_name = self.model.names[cls_id]
                    bbox.confidence = float(conf)
                    bbox.xmin = int(xyxy[0])
                    bbox.ymin = int(xyxy[1])
                    bbox.xmax = int(xyxy[2])
                    bbox.ymax = int(xyxy[3])
                    bbox_array.boxes.append(bbox)

            # 发布检测结果数组
            self.bbox_pub.publish(bbox_array)

            # 性能监控
            process_fps = 1 / (time.time() - start_time + 1e-6)
            self.get_logger().info(
                f"处理帧率: {process_fps:.1f}FPS | 检测到 {len(bbox_array.boxes)} 个目标",
                throttle_duration_sec=1,
            )

            # 本地显示结果
            annotated_frame = results[0].plot()
            cv2.imshow("Local Preview", annotated_frame)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    detector = YOLODetector()

    try:
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(detector)
        executor.spin()
    except KeyboardInterrupt:
        detector.get_logger().info("检测节点关闭")
    finally:
        detector.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
