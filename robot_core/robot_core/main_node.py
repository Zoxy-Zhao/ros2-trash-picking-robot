import rclpy
from rclpy.executors import MultiThreadedExecutor
from robot_core.detection_subscriber import DetectionSubscriber  # 导入节点类

def main():
    rclpy.init()
    
    # 创建所有节点实例
    main_node = rclpy.create_node('main_node')
    detection_node = DetectionSubscriber(main_node)
    
    # 配置执行器
    executor = MultiThreadedExecutor()
    executor.add_node(main_node)
    executor.add_node(detection_node)
    
    main_node.get_logger().info("主节点已启动")
    
    try:
        executor.spin()  # 统一管理所有节点
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        rclpy.shutdown()

if __name__ == "__main__":
    main()