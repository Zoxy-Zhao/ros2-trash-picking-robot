"""Asynchronous service client; never recursively spins an executor."""
from interfaces.srv import ArmControl, Boolean, Perspective


class Client:
    def __init__(self, node):
        self.node = node
        self.arm_control = node.create_client(ArmControl, 'arm_control')
        self.enable_detection = node.create_client(Boolean, 'enable_detection')
        self.perspective_server = node.create_client(Perspective, 'perspective_service')

    def detection(self, enabled):
        request = Boolean.Request()
        request.status = enabled
        return self.enable_detection.call_async(request)

    def perspective(self, x, y):
        request = Perspective.Request()
        request.x, request.y = float(x), float(y)
        return self.perspective_server.call_async(request)

    def fetch(self, x, y, z, class_name, rpy):
        request = ArmControl.Request()
        request.type = 'fetch'
        request.x, request.y, request.z = float(x), float(y), float(z)
        request.roll, request.pitch, request.yaw = map(float, rpy)
        request.class_name = class_name
        return self.arm_control.call_async(request)
