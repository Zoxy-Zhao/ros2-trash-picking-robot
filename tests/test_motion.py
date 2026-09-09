import sys
from pathlib import Path
import unittest
from types import SimpleNamespace
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'robot_arm'), str(ROOT / 'robot_core')]
from robot_arm.six_axis import SixAxisIK, Geometry, rpy_matrix
from robot_arm.move_joints import trajectory, MoveJoints
from robot_core.target_gate import TargetGate


class KinematicsTests(unittest.TestCase):
    def test_known_zero_pose(self):
        t = SixAxisIK().fk([0]*6)
        np.testing.assert_allclose(t[:3, 3], [19.5, 0, 20.])
        np.testing.assert_allclose(t[:3, :3], np.eye(3))

    def test_known_shoulder_rotation(self):
        t = SixAxisIK().fk([0, 90, 0, 0, 0, 0])
        np.testing.assert_allclose(t[:3, 3], [5, 0, -4.5], atol=1e-10)

    def test_random_pose_round_trips(self):
        rng = np.random.default_rng(20260909)
        solver = SixAxisIK()
        for q in rng.uniform(-170, 170, (300, 6)):
            target = solver.fk(q)
            answer = solver.solve(target[:3, 3], target[:3, :3], np.zeros(6))
            self.assertIsNotNone(answer)
            np.testing.assert_allclose(solver.fk(answer), target, atol=1e-6)

    def test_singular_wrist(self):
        solver = SixAxisIK()
        for angle in [0, 180, -180, 1e-7]:
            q = [25, -40, 70, 35, angle, -15]
            t = solver.fk(q)
            answer = solver.solve(t[:3, 3], t[:3, :3], q)
            self.assertIsNotNone(answer)
            np.testing.assert_allclose(solver.fk(answer), t, atol=1e-6)

    def test_joint_limits_and_seed(self):
        limits = [[-80, 80]]*6
        solver = SixAxisIK(limits=limits)
        q = [20, -30, 40, 10, 35, 20]
        t = solver.fk(q)
        answers = solver.solutions(t[:3, 3], t[:3, :3], q)
        self.assertTrue(answers)
        np.testing.assert_allclose(answers[0], q, atol=1e-6)
        for answer in answers:
            self.assertTrue(np.all(np.abs(answer) <= 80))

    def test_unreachable_and_invalid(self):
        solver = SixAxisIK()
        self.assertIsNone(solver.solve([1000, 0, 0], np.eye(3)))
        with self.assertRaises(ValueError):
            solver.solve([0, 0, 0], np.zeros((3, 3)))
        with self.assertRaises(ValueError):
            solver.solve([float('nan'), 0, 0], np.eye(3))
        with self.assertRaises(ValueError):
            SixAxisIK(Geometry(upper_arm=-1))

    def test_reference_grasp_waypoints(self):
        solver = SixAxisIK()
        seed = [0, -45, 90, 0, 45, 0]
        for point in [[12, 0, 3], [12, 0, 0], [12, 0, 3], [12, 10, 13], [12, 10, 10]]:
            seed = solver.solve(point, rpy_matrix(0, 180, 0), seed)
            self.assertIsNotNone(seed)


class TrajectoryTests(unittest.TestCase):
    def test_limits_endpoints_and_synchronization(self):
        for dof in [4, 6]:
            start, end = np.zeros(dof), np.arange(dof)*20-30
            samples = list(trajectory(start, end, 25, 45, .01))
            values = np.array([v for _, v in samples])
            np.testing.assert_allclose(values[0], start)
            np.testing.assert_allclose(values[-1], end)
            self.assertLessEqual(np.abs(np.diff(values, axis=0)/.01).max(), 25.0001)
            self.assertLessEqual(np.abs(np.diff(values, n=2, axis=0)/.01**2).max(), 45.0001)
            self.assertTrue(np.all(values >= np.minimum(start, end)-1e-8))
            self.assertTrue(np.all(values <= np.maximum(start, end)+1e-8))

    def test_invalid_trajectory(self):
        for args in [([0]*4, [0]*6), ([0]*6, [float('nan')]*6)]:
            with self.assertRaises(ValueError):
                list(trajectory(*args))
        with self.assertRaises(ValueError):
            list(trajectory([0]*6, [1]*6, 0))

    def test_protocol(self):
        self.assertEqual(MoveJoints.command([0]*6), 'ARM6 0 0 0 0 0 0')
        self.assertEqual(MoveJoints.command([0]*4), 'ARM 0 0 0 0')
        with self.assertRaises(ValueError):
            MoveJoints.command([float('inf')]*6)


def box(name='dry', x=0, conf=.9):
    return SimpleNamespace(class_name=name, confidence=conf, xmin=x, xmax=x+10, ymin=0, ymax=10)


class ConfirmationTests(unittest.TestCase):
    def test_consecutive_frames(self):
        gate = TargetGate(frames=3)
        self.assertIsNone(gate.update([box()]))
        self.assertIsNone(gate.update([box()]))
        self.assertIsNotNone(gate.update([box()]))

    def test_empty_class_change_and_position_jump_reset(self):
        for interruption in [[], [box('wet')], [box(x=100)], [box(conf=.3)]]:
            gate = TargetGate(frames=3)
            gate.update([box()])
            gate.update([box()])
            self.assertIsNone(gate.update(interruption))
            self.assertIsNone(gate.update([box()]))


if __name__ == '__main__':
    unittest.main()
