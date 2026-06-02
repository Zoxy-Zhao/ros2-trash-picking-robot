# 逆运动学
import numpy as np

class IKService():
    def __init__(self):
        # self.l1 = 8.2
        self.l1 = 15
        self.l2 = 10.4
        self.l3 = 9.1
        self.l4 = 18.4

    def ik_solution(self, x ,y, z):
        target = [x, y, z]
        success = False

        for angle in range(100, 0, -10):
            if self.calculate_angles(self.l1, self.l2, self.l3, self.l4, angle, target) is not False:
                theta1, theta2, theta3, theta4 = self.calculate_angles(self.l1, self.l2, self.l3, self.l4, angle, target)
                success = True
                break
            
        if success is False:
            return None

        return [theta1, theta2, theta3, theta4]

    def calculate_angles(self, l1, l2, l3, l4, angle, target):
        # 修改参考对象
        target_ = self.rotate_point(target, 180 - angle, l4)
        
        x, y, z = target_
        theta1 = np.arctan2(y, x)
        
        r = np.sqrt(x**2 + y**2)
        z_offset = z - l1
        d = np.sqrt(r**2 + z_offset**2)
        
        if d > (l2 + l3) or d < abs(l2 - l3):
            # 目标超出机械臂的工作范围
            return False
        
        theta2 = np.arctan2(z_offset, r) + np.arccos(
            (l2**2 + d**2 - l3**2) / (2 * l2 * d)
        )
        theta3 = np.arccos(
            (l2**2 + l3**2 - d**2) / (2 * l2 * l3)
        ) - np.pi
        
        # 正运动学计算关节坐标
        joint = [
            l2 * np.cos(theta2) * np.cos(theta1),
            l2 * np.cos(theta2) * np.sin(theta1),
            l1 + l2 * np.sin(theta2)
        ]
        
        # 计算θ4
        theta4_degrees = self.calculate_angle(joint, target_, target)

        if theta4_degrees < 0:
            return False
        
        return np.degrees(theta1), np.degrees(theta2), np.degrees(theta3), theta4_degrees

    def rotate_point(self, a, angle, l):
        angle_rad = np.radians(angle)
        ax, ay, az = a
        
        a_xy = np.array([ax, ay, 0.0], dtype=float)
        norm_xy = np.linalg.norm(a_xy)
        
        if norm_xy == 0:
            u = np.array([1.0, 0.0, 0.0], dtype=float)
        else:
            u = a_xy / norm_xy
        
        k = np.array([0.0, 0.0, 1.0], dtype=float)
        direction = np.cos(angle_rad)*u + np.sin(angle_rad)*k
        return np.array(a) + l * direction

    def calculate_angle(self, A, B, C):
        A = np.array(A)
        B = np.array(B)
        C = np.array(C)
        
        AB = B - A
        BC = C - B
        
        # 计算点积和角度（无符号）
        cos_theta = np.dot(AB, BC) / (np.linalg.norm(AB) * np.linalg.norm(BC))
        cos_theta = np.clip(cos_theta, -1.0, 1.0)
        angle = np.degrees(np.arccos(cos_theta))
        
        # 建立局部坐标系：以AB为z轴
        z_axis = AB / np.linalg.norm(AB)
        
        # 选择一个不平行于AB的参考向量（通常用x轴或y轴）
        ref_vec = np.array([1.0, 0.0, 0.0])
        if np.abs(np.dot(z_axis, ref_vec)) > 0.9:  # 如果AB几乎平行于x轴
            ref_vec = np.array([0.0, 1.0, 0.0])   # 改用y轴
        
        # 计算x轴（垂直于AB和ref_vec）
        x_axis = np.cross(ref_vec, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)
        
        # 计算y轴（完成右手坐标系）
        y_axis = np.cross(z_axis, x_axis)
        
        # 将BC投影到局部xy平面
        BC_proj = BC - np.dot(BC, z_axis) * z_axis
        
        # 计算BC在局部坐标系中的角度
        angle_xy = np.arctan2(np.dot(BC_proj, y_axis), np.dot(BC_proj, x_axis))
        
        # 反转符号：顺时针为负，逆时针为正
        if angle_xy > 0:
            angle = -angle  # 顺时针方向为负
        else:
            angle = angle   # 逆时针方向为正
        
        return angle

