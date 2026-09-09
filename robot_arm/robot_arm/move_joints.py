"""Synchronized quintic joint-space trajectories for four or six axes."""
import time
import numpy as np


def trajectory(starts, targets, max_velocity=50., max_acceleration=100., dt=0.1):
    starts, targets = np.asarray(starts, dtype=float), np.asarray(targets, dtype=float)
    if (starts.shape not in ((4,), (6,)) or targets.shape != starts.shape
            or not np.all(np.isfinite([starts, targets]))):
        raise ValueError('Expected matching finite 4- or 6-axis vectors')
    if not np.all(np.isfinite([max_velocity, max_acceleration, dt])) or min(max_velocity, max_acceleration, dt) <= 0:
        raise ValueError('Velocity, acceleration and dt must be positive')
    distance = float(np.max(np.abs(targets-starts)))
    duration = max(dt, 1.875*distance/max_velocity, np.sqrt(5.774*distance/max_acceleration))
    steps = int(np.ceil(duration/dt))
    for i in range(steps + 1):
        s = i/steps
        blend = 10*s**3 - 15*s**4 + 6*s**5
        yield i*dt, (starts + blend*(targets-starts)).tolist()


class MoveJoints:
    def __init__(self, node):
        self.node = node

    def move_joints(self, starts, targets, speed=5):
        begin = time.monotonic()
        for when, angles in trajectory(starts, targets, max_velocity=speed/0.1):
            if not getattr(self.node, "dry_run", False):
                time.sleep(max(0., begin + when - time.monotonic()))
            if self.node.uart_send(self.command(angles)) is False:
                return False
        return True

    @staticmethod
    def command(angles):
        values = np.asarray(angles, dtype=float)
        if values.shape not in ((4,), (6,)) or not np.all(np.isfinite(values)):
            raise ValueError('Expected four or six finite angles')
        if np.any(np.abs(values) > 360):
            raise ValueError('UART joint angle outside supported range')
        prefix = 'ARM6' if len(values) == 6 else 'ARM'
        return prefix + ' ' + ' '.join(str(int(round(v))) for v in values)
