"""ROS 2 grasp service with preflight IK and acknowledged serial writes."""
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from interfaces.srv import ArmControl, SendString
from robot_arm.move_joints import MoveJoints
from robot_arm.ik_service import IKService
from robot_arm.six_axis import Geometry, SixAxisIK, rpy_matrix


class ArmControlNode(Node):
    def __init__(self):
        super().__init__('arm_control')
        defaults = {
            'arm_dof': 6, 'dry_run': True, 'hardware_calibrated': False,
            'link_dimensions_cm': [15., 10.4, 9.1, 5.],
            'joint_limits_deg': [-180., 180.] * 6,
            'home_deg': [0., -45., 90., 0., 45., 0.],
            'dry_bin_cm': [12., 10., 10.], 'wet_bin_cm': [12., -10., 10.],
            'dry_classes': ['dry', '干垃圾'], 'wet_classes': ['wet', '湿垃圾'],
            'approach_height_cm': 3., 'step_degrees': 5., 'gripper_delay_sec': 1.,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.dof = self.get_parameter('arm_dof').value
        if self.dof not in (4, 6):
            raise ValueError('arm_dof must be 4 or 6')
        self.dry_run = self.get_parameter('dry_run').value
        if not self.dry_run and not self.get_parameter('hardware_calibrated').value:
            raise ValueError('Calibrate joints, bins and home; then set hardware_calibrated=true')
        dims = self.get_parameter('link_dimensions_cm').value
        limits = np.array(self.get_parameter('joint_limits_deg').value).reshape(6, 2)
        self.ik_server = SixAxisIK(Geometry(*dims), limits) if self.dof == 6 else IKService()
        self.home = list(self.get_parameter('home_deg').value) if self.dof == 6 else [0., 150., -90., 60.]
        if len(self.home) != self.dof or not np.all(np.isfinite(self.home)):
            raise ValueError('Invalid home joints')
        if self.dof == 6 and (np.any(self.home < limits[:, 0]) or np.any(self.home > limits[:, 1])):
            raise ValueError('Home violates joint limits')
        self.theta_now = self.home.copy()  # Command estimate, NOT encoder feedback.
        self.faulted = False
        self.move_joints = MoveJoints(self)
        self.uart_client = self.create_client(SendString, 'send_uart_message', callback_group=ReentrantCallbackGroup())
        self.server = self.create_service(ArmControl, 'arm_control', self.arm_control_server)
        self.get_logger().info(f'{self.dof}-axis arm; dry_run={self.dry_run}')

    def arm_control_server(self, request, response):
        response.success = False
        try:
            if self.faulted:
                raise RuntimeError('Previous transport failure: re-home before restarting arm node')
            if request.type != 'fetch':
                raise ValueError('Only fetch requests are supported')
            response.success = self.fetch_object(request)
        except (ValueError, RuntimeError) as exc:
            self.get_logger().error(str(exc))
        return response

    def fetch_object(self, request):
        target = np.array([request.x, request.y, request.z], dtype=float)
        rotation = rpy_matrix(request.roll, request.pitch, request.yaw)
        kind = request.class_name
        if kind in self.get_parameter('dry_classes').value:
            bin_point = np.array(self.get_parameter('dry_bin_cm').value, dtype=float)
        elif kind in self.get_parameter('wet_classes').value:
            bin_point = np.array(self.get_parameter('wet_bin_cm').value, dtype=float)
        else:
            raise ValueError(f'Unmapped class: {kind!r}; configure dry_classes/wet_classes')
        height = float(self.get_parameter('approach_height_cm').value)
        speed = float(self.get_parameter('step_degrees').value)
        delay = float(self.get_parameter('gripper_delay_sec').value)
        if not np.all(np.isfinite([height, speed, delay])) or height <= 0 or speed <= 0 or delay < 0:
            raise ValueError('Invalid motion timing/clearance parameters')
        if target.shape != (3,) or bin_point.shape != (3,) or not np.all(np.isfinite([target, bin_point])):
            raise ValueError('Invalid target/bin coordinates')
        above = target + [0., 0., height]
        bin_above = bin_point + [0., 0., height]
        plan = []
        seed = self.theta_now
        for point in [above, target, above, bin_above, bin_point, bin_above]:
            joints = (self.ik_server.solve(point, rotation, seed) if self.dof == 6
                      else self.ik_server.ik_solution(*point))
            if joints is None or not np.all(np.isfinite(joints)):
                raise ValueError(f'Unreachable waypoint: {point.tolist()}')
            plan.append(joints)
            seed = joints
        plan.append(self.home.copy())
        # Every waypoint must solve before opening the gripper or moving.
        if not self.uart_send('CLAW 0'):
            return False
        for index, joints in enumerate(plan):
            if not self.move_joints.move_joints(self.theta_now, joints, speed):
                self.faulted = True
                return False
            self.theta_now = list(joints)
            if index in (1, 4):
                if not self.dry_run:
                    time.sleep(delay)
                if not self.uart_send('CLAW 1' if index == 1 else 'CLAW 0'):
                    return False
                if not self.dry_run:
                    time.sleep(delay)
        self.get_logger().info('Grasp command sequence completed (no measured joint feedback)')
        return True

    def uart_send(self, string):
        if self.dry_run:
            self.get_logger().debug(f'DRY RUN: {string}')
            return True
        if not self.uart_client.wait_for_service(timeout_sec=2.):
            self.faulted = True
            return False
        request = SendString.Request()
        request.data = string
        future = self.uart_client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(3.):
            self.uart_client.remove_pending_request(future)
            self.faulted = True
            return False
        try:
            result = future.result()
            ok = result is not None and result.success
        except Exception:
            ok = False
        self.faulted = not ok
        return ok


def main(args=None):
    rclpy.init(args=args)
    node = ArmControlNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
