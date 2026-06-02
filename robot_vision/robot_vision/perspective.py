import cv2
import numpy as np
import time
from interfaces.srv import Perspective
import rclpy
from rclpy.node import Node

WIDTH = 960
HEIGHT = 540

SIZE = 130
CENTER_PIX = 200
CENTER_REAL = 20.6
BLOCK_R = 4.75

class PerspectiveNode(Node):
    def __init__(self):
        super().__init__('perspective_service')
        self.srv = self.create_service(
            Perspective, 
            'perspective_service', 
            self.perspective_callback
        )
        self.load_transform_matrix("/home/orin/WorkSpace/robot_ws/src/robot_vision/data/matrix.npy")

        self.get_logger().info("透视变换服务已就绪")

    def load_transform_matrix(self, filename):
        """
        从.npy文件加载透视变换矩阵。
        """
        try:
            self.transform_matrix = np.load(filename)
            print(f"透视变换矩阵已从 {filename} 加载")
        except Exception as e:
            print(f"加载矩阵失败: {e}")
    
    def transform_point(self, src_point, transform_matrix):
        """
        使用透视变换矩阵将单个坐标转换为透视变换后的坐标。
        """
        # 将原始坐标转换为齐次坐标 (x, y, 1)
        src_point_homogeneous = np.array([src_point[0], src_point[1], 1])

        # 使用透视变换矩阵进行转换
        transformed_point = np.dot(transform_matrix, src_point_homogeneous)

        # 转换后的坐标需要归一化 (除以 w')
        x_prime = transformed_point[0] / transformed_point[2]
        y_prime = transformed_point[1] / transformed_point[2]

        return (x_prime, y_prime)
    
    def to_axis(self, position):
        """通过平视摄像头的坐标来推算出物品的实际坐标(cm)"""
        # 坐标转换
        x = HEIGHT - position[1]
        y = position[0]

        # 计算单位长度 (pix / cm)
        unit = SIZE / BLOCK_R
        real_x = CENTER_REAL + (x - (HEIGHT - CENTER_PIX)) / unit
        real_y = (y - WIDTH / 2) / unit

        return (real_x, real_y)


    def perspective_callback(self, request, response):
        result = self.transform_point((request.x, request.y), self.transform_matrix)
        result = self.to_axis(result)
        response.x_real = float(result[0])
        response.y_real = float(result[1])
        self.get_logger().info(f"result: X:{result[0]}, Y:{result[1]}")
        response.success = True
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = PerspectiveNode()
    rclpy.spin(node)
    rclpy.shutdown()


class PerspectiveTransforms:
    
    def __init__(self):
        """
        初始化 PerspectiveCalibration 类的实例。
        """
        self.transform_matrix = None

    def save_transform_matrix(self, filename):
        """
        将透视变换矩阵保存为.npy文件。
        """
        if self.transform_matrix is not None:
            np.save(filename, self.transform_matrix)  # 将矩阵保存为.npy文件
            print(f"透视变换矩阵已保存为 {filename}")
        else:
            print("没有计算出透视变换矩阵，无法保存！")

    def detect_diamond_corners(self, frame):
        """
        检测图像中菱形的四个角并返回坐标，确保角点顺序为：上、右、下、左。
        """
        # 转换为灰度图像
        gray_img = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        # 二值化处理（根据实际情况调整阈值）
        _, thresh_img = cv2.threshold(gray_img, 100, 255, cv2.THRESH_BINARY_INV)

        cv2.imshow("threshold", thresh_img)
        cv2.waitKey()

        # 查找轮廓
        contours, _ = cv2.findContours(thresh_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 找到最大的轮廓（假设菱形是图像中最大的轮廓）
        max_contour = max(contours, key=cv2.contourArea)

        # 获取菱形的四个角点
        epsilon = 0.02 * cv2.arcLength(max_contour, True)  # 逼近精度
        approx = cv2.approxPolyDP(max_contour, epsilon, True)

        # 如果检测到的点数不是4个，返回空列表或报错
        if len(approx) != 4:
            raise ValueError("未检测到四个角点")

        # 计算四个点的中心
        center = np.mean(approx, axis=0)[0]

        # 计算每个点与中心的角度
        def angle_from_center(point):
            x, y = point
            cx, cy = center
            return np.arctan2(y - cy, x - cx)

        # 对四个角点按角度排序
        sorted_corners = sorted(approx, key=lambda point: angle_from_center(point[0]))

        # 返回按顺序排列的角点
        return [(point[0][0], point[0][1]) for point in sorted_corners]

    def get_MM(self, srcArr_list, dstArr_list):
        """
        计算透视变换矩阵。
        """
        # 获取原始角点坐标（源点）
        srcArr = np.float32(srcArr_list)

        # 设置目标角点（即透视变换后的目标位置）
        dstArr = np.float32(dstArr_list)

        # 计算透视变换矩阵
        MM = cv2.getPerspectiveTransform(srcArr, dstArr)

        return MM

    def calibration_perspective(self, frame):
        """校准透视变换"""
        cv2.imshow("Original", frame)
        cv2.waitKey()
        # 检测菱形角点
        try:
            diamond_corners = self.detect_diamond_corners(frame)
            height, width = frame.shape[:2]

            # 设置目标角点
            srcArr_list = diamond_corners
            dstArr_list = [
                [width / 2 - SIZE, CENTER_PIX],
                [width / 2, CENTER_PIX - SIZE],
                [width / 2 + SIZE, CENTER_PIX],
                [width / 2, CENTER_PIX + SIZE]
            ]

            # 获取透视变换矩阵
            self.transform_matrix = self.get_MM(srcArr_list, dstArr_list)

            # 透视变换
            dst = cv2.warpPerspective(frame, self.transform_matrix, (width, height))

            # 显示原始图像和透视变换后的图像
            
            cv2.imshow("Warped", dst)
            cv2.waitKey(0)
        except ValueError as e:
            print(f"错误: {e}")

        # 关闭所有窗口
        cv2.destroyAllWindows()
        self.save_transform_matrix("/home/orin/WorkSpace/robot_ws/src/robot_vision/data/matrix.npy")
    
def gstreamer_pipeline(
        sensor_id=0,
        capture_width=1280,
        capture_height=720,
        display_width=WIDTH,   # 建议与捕获分辨率一致以避免缩放
        display_height=HEIGHT,
        framerate=30,    
        flip_method=0,    
    ):
        return (
            "nvarguscamerasrc sensor-id=%d ! "
            "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
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

if __name__ == "__main__":
    # 创建校准对象
    calibration = PerspectiveTransforms()
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

    if not video_capture.isOpened():
        print("无法打开摄像头。")

    now_time = time.time()

    while True:
        ret_val, frame = video_capture.read()
        cv2.imshow("input", frame)
        keyCode = cv2.waitKey(30) & 0xFF        
        # 按下esc暂停
        if keyCode == '24':
            break 
        # 使用时间来暂停
        # if time.time() - now_time > 3:
        #     break

    
    # 执行透视校准
    calibration.calibration_perspective(frame)
