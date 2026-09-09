"""Latest-frame detection with explicit PyTorch/TensorRT backend selection."""
import os
import threading
import time
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from interfaces.msg import BoundingBox, BoundingBoxArray
from interfaces.srv import Boolean
from robot_vision.inference import Detector


class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        for name, default in {
            'model_path': os.path.expanduser('~/robot_ws/models/best.engine'),
            'model_family': 'yolov5', 'yolov5_repo': os.path.expanduser('~/yolov5'),
            'device': '0', 'imgsz': 640, 'display': False,
        }.items():
            self.declare_parameter(name, default)
        self.detector = Detector(self.get_parameter('model_path').value,
                                 self.get_parameter('model_family').value,
                                 self.get_parameter('yolov5_repo').value,
                                 self.get_parameter('device').value,
                                 self.get_parameter('imgsz').value)
        self.display = self.get_parameter('display').value
        self.bridge = CvBridge()
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.enable = True
        self.generation = 0
        self.stop = threading.Event()
        self.ready = threading.Event()
        self.subscription = self.create_subscription(Image, 'camera/image', self.image_callback,
                                                       rclpy.qos.qos_profile_sensor_data)
        self.server = self.create_service(Boolean, 'enable_detection', self.enable_detection)
        self.bbox_pub = self.create_publisher(BoundingBoxArray, 'detection_boxes', 10)
        self.process_thread = threading.Thread(target=self.process_frame, daemon=True)
        self.process_thread.start()

    def enable_detection(self, request, response):
        with self.frame_lock:
            self.enable = request.status
            self.generation += 1
            self.latest_frame = None
        response.success = True
        return response

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self.frame_lock:
                if not self.enable:
                    return
                self.latest_frame = (frame, msg.header, self.generation)
            self.ready.set()
        except Exception as exc:
            self.get_logger().error(str(exc))

    def process_frame(self):
        while not self.stop.is_set():
            self.ready.wait(0.1)
            self.ready.clear()
            with self.frame_lock:
                item, self.latest_frame = self.latest_frame, None
            if item is None:
                continue
            frame, header, generation = item
            started = time.perf_counter()
            try:
                records = self.detector.predict(frame)
                message = BoundingBoxArray()
                message.header = header
                for name, conf, x1, y1, x2, y2 in records:
                    box = BoundingBox()
                    box.class_name, box.confidence = name, conf
                    box.xmin, box.ymin, box.xmax, box.ymax = x1, y1, x2, y2
                    message.boxes.append(box)
                    if self.display:
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                with self.frame_lock:
                    if not self.enable or generation != self.generation:
                        continue
                    self.bbox_pub.publish(message)
                latency_ms = (time.perf_counter()-started)*1000
                self.get_logger().info(f'Detection processing: {latency_ms:.1f} ms', throttle_duration_sec=2.)
                if self.display:
                    cv2.imshow('Detection', frame)
                    cv2.waitKey(1)
            except Exception as exc:
                self.get_logger().error(f'Inference stopped: {exc}')
                self.stop.set()

    def destroy_node(self):
        self.stop.set()
        self.ready.set()
        self.process_thread.join(timeout=5.)
        if self.display:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YOLODetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
