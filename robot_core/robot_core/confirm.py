"""Non-blocking detect/pause/localize/grasp/resume state machine."""
import time
from robot_core.client import Client
from robot_core.target_gate import TargetGate


class Confirm:
    def __init__(self, node):
        self.node = node
        self.client = Client(node)
        self.gate = TargetGate()
        self.stage = 'idle'
        self.pending = None
        node.declare_parameter('grasp_z_cm', 0.)
        node.declare_parameter('grasp_rpy_deg', [0., 180., 0.])
        node.declare_parameter('grasp_timeout_sec', 120.)
        self.timer = node.create_timer(0.05, self.tick)

    def confirm_flush(self, boxes, stamp_ns=None):
        if self.stage != 'idle':
            return
        box = self.gate.update(boxes, stamp_ns)
        if box is None:
            return
        services = (self.client.arm_control, self.client.enable_detection, self.client.perspective_server)
        if not all(s.service_is_ready() for s in services):
            self.node.get_logger().warning('Grasp services not ready')
            return
        self.box = box
        self.submit('pausing', self.client.detection(False), 5.)

    def submit(self, stage, future, timeout):
        self.stage, self.pending = stage, future
        self.deadline = time.monotonic() + timeout

    def resume(self):
        self.submit('resuming', self.client.detection(True), 5.)

    def tick(self):
        if self.pending is None:
            return
        if not self.pending.done():
            if time.monotonic() < self.deadline:
                return
            # A timed-out arm request can still be executing. Latch fault to
            # avoid scheduling another grasp over an unknown physical state.
            self.pending = None
            self.stage = 'fault'
            self.node.get_logger().error('Service timeout; execution state unknown. Restart after checking robot.')
            return
        try:
            result = self.pending.result()
            self.pending = None
            if result is None or not result.success:
                raise RuntimeError(f'{self.stage} service failed')
            if self.stage == 'pausing':
                b = self.box
                self.submit('localizing', self.client.perspective((b.xmin+b.xmax)/2, (b.ymin+b.ymax)/2), 5.)
            elif self.stage == 'localizing':
                self.submit('grasping', self.client.fetch(result.x_real, result.y_real,
                            self.node.get_parameter('grasp_z_cm').value, self.box.class_name,
                            self.node.get_parameter('grasp_rpy_deg').value),
                            self.node.get_parameter('grasp_timeout_sec').value)
            elif self.stage == 'grasping':
                self.resume()
            elif self.stage == 'resuming':
                self.stage = 'idle'
                self.gate.reset()
        except Exception as exc:
            self.node.get_logger().error(str(exc))
            if self.stage in ('pausing', 'localizing'):
                self.resume()
            else:
                self.stage, self.pending = 'fault', None
