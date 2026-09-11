import copy
import json
from pathlib import Path
import shutil
import unittest

import numpy as np
from showcase.pipeline import (ROOT, load_config, run, SimReceiver, verify_firmware,
                               SixAxisIK, rpy_matrix, plan_grasp, PlaneProjector, TargetGate)
from robot_vision.plane_geometry import PlaneCalibration
from types import SimpleNamespace


def example():
    return json.loads((ROOT/'showcase/example.json').read_text(encoding='utf-8'))


class SharedProjectionTests(unittest.TestCase):
    def test_matches_calibrated_coordinate_formula(self):
        projector = PlaneProjector(np.eye(3))
        np.testing.assert_allclose(projector.to_base_xy([480, 200]), [20.6, 0])
        np.testing.assert_allclose(projector.to_base_xy(example()['pixel_center']), [12, 0])
        # Perspective normalization is applied before pixel-to-cm scaling.
        h = [[2, 0, 0], [0, 2, 0], [0, 0, 2]]
        np.testing.assert_allclose(PlaneProjector(h).to_base_xy([480, 200]), [20.6, 0])

    def test_invalid_and_projective_infinity(self):
        for matrix in [np.zeros((3, 3)), [[1, 2]], np.full((3, 3), np.nan)]:
            with self.assertRaises(ValueError):
                PlaneProjector(matrix)
        with self.assertRaises(ValueError):
            PlaneProjector(np.eye(3), PlaneCalibration(diamond_px=0))
        with self.assertRaises(ValueError):
            PlaneProjector([[1,0,0],[0,1,0],[1,0,-1]]).to_base_xy([1, 1])


class FreshFrameTests(unittest.TestCase):
    def test_duplicates_and_out_of_order_do_not_confirm(self):
        b = SimpleNamespace(confidence=.9, xmin=0, xmax=10, ymin=0, ymax=10, class_name='dry')
        gate = TargetGate(frames=3)
        for stamp in [100,100,99,100,0]:
            self.assertIsNone(gate.update([b], stamp))
        self.assertEqual(gate.count, 1)
        self.assertIsNone(gate.update([b], 101))
        self.assertIsNotNone(gate.update([b], 102))
        self.assertIsNone(gate.update([b], 102))
        self.assertEqual(gate.count, 0)


class SharedPlanTests(unittest.TestCase):
    def test_lift_clears_target_and_bin_and_gripper_is_separate(self):
        solver = SixAxisIK()
        home = [0, -45, 90, 0, 45, 0]
        phases = plan_grasp(solver, [12,0,0], rpy_matrix(0,180,0), [12,10,10], home, home)
        self.assertEqual([p.name for p in phases], ['approach','grasp','lift','transfer','place','retreat','home'])
        self.assertEqual(phases[2].tcp_cm[2], 13)
        self.assertEqual(phases[3].tcp_cm[2], 13)
        self.assertEqual([p.gripper_after for p in phases if p.gripper_after], ['CLAW 1','CLAW 0'])
        for p in phases:
            self.assertEqual(len(p.joints), 6)
            np.testing.assert_allclose(solver.fk(p.joints)[:3,3], p.tcp_cm, atol=1e-6)

    def test_invalid_targets_and_seed(self):
        solver = SixAxisIK()
        for target, seed in [([1000,0,0], [0]*6), ([np.nan,0,0],[0]*6), ([12,0,0],[181]*6)]:
            with self.assertRaises(ValueError):
                plan_grasp(solver,target,rpy_matrix(0,180,0),[12,10,10],[0]*6,seed)


class PipelineTests(unittest.TestCase):
    def test_both_classes_complete_and_return_home(self):
        for kind in ['dry','wet']:
            e=example(); e['class_name']=kind
            result=run(e)
            self.assertEqual(result['status'], 'DONE')
            self.assertEqual([s for s in result['uart_commands'] if s.startswith('CLAW')], ['CLAW 0','CLAW 1','CLAW 0'])
            np.testing.assert_allclose(result['final_commanded_deg'], load_config()[0]['home_deg'])
            self.assertEqual(result['final_gripper'], 0)
            self.assertGreater(result['max_quantization_tcp_error_cm'], 0)
            self.assertLess(result['max_quantization_tcp_error_cm'], 1.)
            self.assertTrue(all(len(t['commanded_deg'])==6 for t in result['trace']))

    def test_preflight_and_confirmation_fail_without_commands(self):
        for fault in ['stale','unreachable']:
            result=run(example(),fault=fault)
            self.assertEqual(result['status'], 'HALTED')
            self.assertEqual(result['uart_commands'], [])
        e=example(); e['class_name']='unknown'
        self.assertEqual(run(e)['fault'], 'UNMAPPED_CLASS')

    def test_transport_and_estop_stop_without_return_or_open(self):
        for fault in ['transport','estop']:
            r=run(example(),fault=fault)
            self.assertEqual(r['status'], 'HALTED')
            self.assertTrue(r['trace'])
            self.assertEqual(r['events'][-1]['state'], 'HALTED')
            self.assertFalse(any(t['phase']=='home' for t in r['trace']))
            self.assertEqual([x for x in r['uart_commands'] if x.startswith('CLAW')], ['CLAW 0'])

    def test_quantized_limit_rejected_before_any_motion(self):
        config = copy.deepcopy(load_config())
        config[0]['home_deg'][0]=.6
        config[0]['joint_limits_deg'][1]=.9
        # A narrow base limit may reject a bin before quantization, both must be zero-send.
        r=run(example(),config=config)
        self.assertEqual(r['status'], 'HALTED')
        self.assertFalse(r['uart_commands'])

    def test_receiver_refuses_old_arm_and_invalid_frames_latches(self):
        for command in ['ARM 0 0 0 0', 'ARM6 1 2 3', 'CLAW 2', 'ARM6 nan 0 0 0 0 0', 'CLAW 1\n']:
            r=SimReceiver(SixAxisIK(),[0]*6)
            with self.assertRaises(ValueError): r.accept(command)
            with self.assertRaises(ValueError): r.accept('CLAW 0')
            self.assertFalse(r.frames)

    @unittest.skipUnless(shutil.which('gcc'), 'Host GCC unavailable')
    def test_full_pipeline_replayed_through_actual_c_parser(self):
        r=run(example(),firmware_check=True)
        self.assertEqual(r['firmware_check']['status'], 'PASSED')
        self.assertEqual(r['firmware_check']['frames'],len(r['uart_commands']))
        self.assertEqual(r['firmware_check']['dma_fragment_bytes'],7)


if __name__=='__main__':
    unittest.main()
