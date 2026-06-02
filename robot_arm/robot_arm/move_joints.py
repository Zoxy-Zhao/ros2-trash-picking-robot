# 关节移动控制
import numpy as np
import time

class MoveJoints():
    def __init__(self, node):
       self.node = node

    def move_joints(self, starts, targets, speed):
        """处理移动请求"""
        nows = starts.copy()
        step = speed
        while not np.allclose(nows, targets, atol=0.1):
            for i in range(4):
                if abs(nows[i] - targets[i]) <= step:
                    nows[i] = targets[i]
                else:
                    if targets[i] > starts[i]:
                        nows[i] += step
                    else:
                        nows[i] -= step
            time.sleep(0.1)
            self.set_servos(nows)
        return True

    def set_servos(self, angles):
        angles_int = [int(round(angle)) for angle in angles]
        uart_msg = f"ARM {' '.join(map(str, angles_int))}\r\n"
        self.node.uart_send(uart_msg)
