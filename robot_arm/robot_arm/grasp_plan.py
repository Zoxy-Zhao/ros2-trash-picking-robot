"""Shared six-axis task planning for ROS service and offline demonstration."""
from dataclasses import dataclass
import numpy as np


@dataclass
class GraspPhase:
    name: str
    purpose: str
    joints: list
    tcp_cm: list
    gripper_after: str = ''


def plan_grasp(solver, target, rotation, bin_point, home, seed, approach_height=3.):
    target, bin_point = np.asarray(target, float), np.asarray(bin_point, float)
    home, seed = np.asarray(home, float), np.asarray(seed, float)
    if target.shape != (3,) or bin_point.shape != (3,) or not np.isfinite([target, bin_point]).all():
        raise ValueError('Expected finite target and bin coordinates')
    if not np.isfinite(approach_height) or approach_height <= 0:
        raise ValueError('Approach height must be positive')
    for q in (home, seed):
        if (q.shape != (6,) or not np.isfinite(q).all() or np.any(q < solver.limits[:, 0])
                or np.any(q > solver.limits[:, 1])):
            raise ValueError('Home/seed violates joint configuration')
    above = target + [0., 0., approach_height]
    clearance = max(target[2], bin_point[2]) + approach_height
    lift = np.r_[target[:2], clearance]
    bin_above = np.r_[bin_point[:2], clearance]
    waypoints = [
        ('approach', '接近目标上方', above, ''),
        ('grasp', '下降抓取并闭爪', target, 'CLAW 1'),
        ('lift', '提升至搬运高度', lift, ''),
        ('transfer', '移至分类桶上方', bin_above, ''),
        ('place', '下降投放并开爪', bin_point, 'CLAW 0'),
        ('retreat', '抬升离开分类桶', bin_above, ''),
    ]
    plan = []
    for name, purpose, point, gripper in waypoints:
        q = solver.solve(point, rotation, seed)
        if q is None:
            raise ValueError(f'Unreachable {name}: {point.tolist()}')
        plan.append(GraspPhase(name, purpose, q, point.tolist(), gripper))
        seed = q
    plan.append(GraspPhase('home', '返回待机位姿', home.tolist(), solver.fk(home)[:3, 3].tolist()))
    return plan
