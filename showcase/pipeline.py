"""Shared modules -> full plan -> quantized UART commands -> software receiver."""
from dataclasses import asdict
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
for package in ('robot_arm', 'robot_core', 'robot_vision'):
    location = str(ROOT / package)
    if location not in sys.path:
        sys.path.insert(0, location)
from robot_arm.six_axis import Geometry, SixAxisIK, rpy_matrix
from robot_arm.grasp_plan import plan_grasp
from robot_arm.move_joints import trajectory, MoveJoints
from robot_core.target_gate import TargetGate
from robot_vision.plane_geometry import PlaneProjector


def load_config(path=None):
    data = yaml.safe_load(Path(path or ROOT/'robot_launch/config/robot.yaml').read_text(encoding='utf-8'))
    return data['arm_control']['ros__parameters'], data['detection_subscriber']['ros__parameters']


def verify_firmware(lines):
    """Replay bytes through actual C parser with mock HAL, not motor firmware."""
    gcc = shutil.which('gcc')
    if gcc is None:
        raise RuntimeError('GCC required for --firmware-check; no C validation was performed')
    with tempfile.TemporaryDirectory() as folder:
        binary = str(Path(folder)/'receiver.exe')
        subprocess.run([gcc, '-std=c11', '-Wall', '-Wextra', '-Werror',
                        '-I'+str(ROOT/'tests/firmware_mocks'),
                        str(ROOT/'firmware/Core/Src/transmit.c'),
                        str(ROOT/'showcase/firmware_receiver.c'), '-o', binary],
                       check=True, capture_output=True, text=True)
        data = ''.join(line+'\r\n' for line in lines)
        output = subprocess.run([binary], input=data, capture_output=True, text=True, check=True)
    decoded = [json.loads(line) for line in output.stdout.splitlines()]
    expected = [{'op': line.split()[0], 'params': [float(v) for v in line.split()[1:]]} for line in lines]
    if decoded != expected:
        raise ValueError('Firmware decoded stream differs from transmitted commands')
    return {'status': 'PASSED', 'frames': len(decoded), 'dma_fragment_bytes': 7,
            'scope': 'Actual transmit.c on host with mock HAL; no PWM/actuator verification'}


class SimReceiver:
    def __init__(self, solver, home):
        self.solver = solver
        self.joints = list(home)
        self.gripper = 0
        self.frames = []
        self.fault = None

    def halt(self, reason):
        self.fault = reason  # freeze the command estimate; no automatic home/open

    def accept(self, line):
        if self.fault:
            raise ValueError('Receiver fault latched')
        tokens = line.split()
        try:
            if '\n' in line or '\r' in line or len(line.encode('ascii')) > 62:
                raise ValueError('Invalid UART frame')
            values = np.asarray([float(v) for v in tokens[1:]])
            if not np.isfinite(values).all():
                raise ValueError('Nonfinite command')
            if tokens[0] == 'ARM6' and len(values) == 6:
                if np.any(values < self.solver.limits[:, 0]) or np.any(values > self.solver.limits[:, 1]):
                    raise ValueError('Command exceeds model limits')
                self.joints = values.tolist()
            elif tokens[0] == 'CLAW' and len(values) == 1 and values[0] in (0, 1):
                self.gripper = int(values[0])
            else:
                raise ValueError('Expected ARM6 six motion joints or independent CLAW')
        except (ValueError, IndexError, UnicodeError) as exc:
            self.halt(str(exc))
            raise ValueError(self.fault) from exc
        self.frames.append(line)


def run(example, config=None, fault=None, firmware_check=False):
    arm, perception = config or load_config()
    if arm['arm_dof'] != 6:
        raise ValueError('Showcase requires six-axis configuration')
    solver = SixAxisIK(Geometry(*arm['link_dimensions_cm']), np.array(arm['joint_limits_deg']).reshape(6, 2))
    receiver = SimReceiver(solver, arm['home_deg'])
    events, trace, phases, confirmation = [], [], [], []
    time_sec = 0.
    xyz = None
    result = {'source': 'SOFTWARE SIMULATION; synthetic detections/calibration. No physical motion or measured performance.',
              'example': example, 'arm_config': arm, 'firmware_check': {'status': 'NOT_RUN'}}

    def state(name):
        events.append({'time_sec': time_sec, 'state': name})

    try:
        state('CONFIRMING')
        gate = TargetGate(frames=example['confirmation_frames'])
        found = None
        for i in range(example['frames']):
            stamp = 1_000_000_000 + i*example['frame_interval_ns']
            if fault == 'stale':
                stamp = 1_000_000_000
            u, v = example['pixel_center']
            box = SimpleNamespace(class_name=example['class_name'], confidence=example['confidence'],
                                  xmin=u-12, xmax=u+12, ymin=v-12, ymax=v+12)
            found = gate.update([box], stamp)
            confirmation.append({'stamp_ns': stamp, 'count': gate.count, 'confirmed': found is not None})
            if found is not None:
                break
        if found is None:
            raise ValueError('TARGET_NOT_CONFIRMED')
        state('LOCALIZING')
        projector = PlaneProjector(example['homography'])
        xy = projector.to_base_xy(example['pixel_center'])
        xyz = [*xy, perception['grasp_z_cm']]
        if fault == 'unreachable':
            xyz[0] = 1000.
        kind = example['class_name']
        if kind in arm['dry_classes']:
            bin_point = arm['dry_bin_cm']
        elif kind in arm['wet_classes']:
            bin_point = arm['wet_bin_cm']
        else:
            raise ValueError('UNMAPPED_CLASS')
        state('PREFLIGHT')
        phases = plan_grasp(solver, xyz, rpy_matrix(*perception['grasp_rpy_deg']), bin_point,
                            arm['home_deg'], arm['home_deg'], arm['approach_height_cm'])
        # Preflight all encoded points, including integer command quantization.
        seed = arm['home_deg']
        prepared = []
        for phase in phases:
            samples = list(trajectory(seed, phase.joints, max_velocity=arm['step_degrees']/.1))
            for when, q in samples:
                encoded = MoveJoints.command(q)
                quantized = np.array([float(v) for v in encoded.split()[1:]])
                if np.any(quantized < solver.limits[:, 0]) or np.any(quantized > solver.limits[:, 1]):
                    raise ValueError('Quantized command exceeds joint limits')
            prepared.append((phase, samples))
            seed = phase.joints
        receiver.accept('CLAW 0')
        for phase, samples in prepared:
            state(phase.name.upper())
            start = time_sec
            for when, q in samples:
                time_sec = start+when
                if fault == 'estop' and time_sec >= .5:
                    raise ValueError('ESTOP_SIMULATED')
                if fault == 'transport' and len(receiver.frames) >= 4:
                    raise ValueError('UART_WRITE_FAILURE_SIMULATED')
                receiver.accept(MoveJoints.command(q))
                actual = solver.fk(receiver.joints)[:3, 3]
                trace.append({'time_sec': time_sec, 'phase': phase.name, 'planned_deg': q,
                              'commanded_deg': receiver.joints.copy(), 'tcp_cm': actual.tolist(),
                              'gripper': receiver.gripper,
                              'quantization_tcp_error_cm': float(np.linalg.norm(actual-solver.fk(q)[:3, 3]))})
            if phase.gripper_after:
                time_sec += arm['gripper_delay_sec']
                receiver.accept(phase.gripper_after)
                time_sec += arm['gripper_delay_sec']
        state('DONE')
    except ValueError as exc:
        receiver.halt(str(exc))
        state('HALTED')
    if firmware_check:
        result['firmware_check'] = verify_firmware(receiver.frames)
    result.update(status='HALTED' if receiver.fault else 'DONE', fault=receiver.fault,
                  target_base_cm=xyz, events=events, confirmation=confirmation,
                  phases=[asdict(p) for p in phases], trace=trace, uart_commands=receiver.frames,
                  final_commanded_deg=receiver.joints, final_gripper=receiver.gripper,
                  virtual_duration_sec=time_sec,
                  max_quantization_tcp_error_cm=max((s['quantization_tcp_error_cm'] for s in trace), default=0))
    return result
