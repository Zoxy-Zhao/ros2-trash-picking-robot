# 确认夹取物体是否合法
import rclpy
from rclpy.parameter import Parameter
from robot_core.client import Client

class Confirm:

    def __init__(self, node):
        self.node = node
        self.client = Client(node)
        self.number = 0

    def confirm_flush(self, boxes):
        """更新信息"""
        self.boxes = boxes
        box = self.get_highest_confidence_box_center(boxes)
        if box is not None:
            self.confirm_object(box)
        
    def confirm_object(self, box): 
        """确认夹取目标"""
        if box.confidence > 0.7 and self.confirm_location(box):
            self.node.get_logger().info("发现高置信度目标")

            #当连续10帧都是高置信度则准备开启夹取
            if self.number > 10:

                self.client.stop_yolo()
                x ,y = self.client.get_perspective((box.xmin + box.xmax) / 2, (box.ymin + box.ymax) / 2)
                self.client.fetch_object(x ,y ,0)
                self.client.start_yolo()

                self.number = 0
                
            else:
                self.number += 1
        else:
            self.number = 0

    def confirm_location(self, box):
        """确认目标位置是否合法"""
        return True

    def get_highest_confidence_box_center(self, boxes):
        """返回置信度最高的框"""
        max_box = None
        confidence = 0
        for i, box in enumerate(boxes, 1):
            if box.confidence > confidence:
                max_box = box
                confidence = box.confidence
        return max_box