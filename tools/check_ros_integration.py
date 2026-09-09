"""Exercise real generated ROS services after colcon build (no hardware)."""
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from interfaces.srv import ArmControl
from robot_arm.arm_control import ArmControlNode


def main():
    rclpy.init()
    arm = ArmControlNode()
    caller = Node('arm_integration_test')
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(arm)
    executor.add_node(caller)
    calls = []
    original = arm.uart_send
    def record(command):
        calls.append(command)
        return original(command)
    arm.uart_send = record
    client = caller.create_client(ArmControl, 'arm_control')
    try:
        assert client.wait_for_service(timeout_sec=5.), 'Arm service not discovered'
        def invoke(x, class_name):
            request = ArmControl.Request()
            request.type, request.class_name = 'fetch', class_name
            request.x, request.y, request.z, request.pitch = x, 0., 0., 180.
            future = client.call_async(request)
            deadline = time.monotonic() + 20.
            while not future.done() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.05)
            assert future.done(), 'Arm request timed out'
            return future.result().success
        assert not invoke(1000., 'dry'), 'Unreachable request succeeded'
        assert not calls, 'Unreachable request emitted commands'
        assert not invoke(12., 'unknown'), 'Unknown class succeeded'
        assert not calls, 'Unknown class emitted commands'
        assert invoke(12., 'dry'), 'Reference grasp failed'
        assert any(c.startswith('ARM6 ') for c in calls)
        assert [c for c in calls if c.startswith('CLAW')] == ['CLAW 0', 'CLAW 1', 'CLAW 0']
        assert arm.theta_now == arm.home
        print('ROS service integration passed: reachable grasp, preflight rejection, six-axis commands')
    finally:
        executor.shutdown()
        caller.destroy_node()
        arm.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
