# detection_subscriber.py
import rclpy
from rclpy.node import Node
from interfaces.msg import BoundingBoxArray 
from rclpy.qos import QoSReliabilityPolicy, QoSProfile
from robot_core.confirm import Confirm

class DetectionSubscriber(Node):
    def __init__(self, node):
        super().__init__('detection_subscriber')

        self.confirm = Confirm(node)
        
        # 配置QoS以匹配发布者设置
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE  # 与发布者保持一致
        )
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            BoundingBoxArray,
            'detection_boxes',
            self.detection_callback,
            qos_profile
        )
        self.subscription  # 防止未使用警告
        
        self.get_logger().info("检测结果订阅节点已启动，等待数据...")

    def detection_callback(self, msg):
        """处理检测结果的回调函数"""
        try:
            # 打印基本信息
            header = msg.header
            timestamp = header.stamp.sec + header.stamp.nanosec * 1e-9
            self.get_logger().info(
                f"\n收到检测结果 [时间戳: {timestamp:.3f}s] [帧ID: {header.frame_id}]\n"
                f"检测到 {len(msg.boxes)} 个目标:"
            )
            
            # 打印每个检测框的详细信息
            for i, box in enumerate(msg.boxes, 1):
                self.get_logger().info(
                    f"目标 {i}:\n"
                    f"  类别: {box.class_name}\n"
                    f"  置信度: {box.confidence:.2%}\n"
                    f"  坐标: ({box.xmin}, {box.ymin}) -> ({box.xmax}, {box.ymax})\n"
                    f"  尺寸: {box.xmax - box.xmin}x{box.ymax - box.ymin} 像素"
                )
            
            self.confirm.confirm_flush(msg.boxes)

        except Exception as e:
            self.get_logger().error(f"处理检测结果时出错: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    subscriber = DetectionSubscriber()
    
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        subscriber.get_logger().info("订阅节点关闭")
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()