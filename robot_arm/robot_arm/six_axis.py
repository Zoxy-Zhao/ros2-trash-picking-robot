"""Analytic IK: Z-Y-Y positioning arm and Z-Y-Z spherical wrist.

Lengths are cm; public angles are degrees. Geometry and joint limits are
configurable. The solver applies to the axis arrangement defined above.
"""
from dataclasses import dataclass
from itertools import product
import numpy as np


def rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])


def ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1., 0], [-s, 0, c]])


def rpy_matrix(roll, pitch, yaw):
    r, p, y = np.radians([roll, pitch, yaw])
    rx = np.array([[1., 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    return rz(y) @ ry(p) @ rx


@dataclass
class Geometry:
    base_height: float = 15.0
    upper_arm: float = 10.4
    forearm: float = 9.1
    tool_length: float = 5.0


class SixAxisIK:
    def __init__(self, geometry=None, limits=None):
        self.geometry = geometry or Geometry()
        lengths = np.array(list(vars(self.geometry).values()))
        if not np.all(np.isfinite(lengths)) or np.any(lengths <= 0):
            raise ValueError('All dimensions must be finite and positive')
        self.limits = np.array(limits if limits is not None else [[-180., 180.]] * 6, dtype=float)
        if (self.limits.shape != (6, 2) or not np.all(np.isfinite(self.limits))
                or np.any(self.limits[:, 0] >= self.limits[:, 1])
                or np.any(self.limits[:, 1] - self.limits[:, 0] > 360)):
            raise ValueError('Expected six ordered limits, span <= 360 degrees')

    def fk(self, joints):
        q = np.radians(joints)
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            raise ValueError('Expected six finite joint angles')
        g = self.geometry
        r02 = rz(q[0]) @ ry(q[1])
        r03 = r02 @ ry(q[2])
        rotation = r03 @ rz(q[3]) @ ry(q[4]) @ rz(q[5])
        wrist = (np.array([0., 0., g.base_height])
                 + r02 @ np.array([g.upper_arm, 0., 0.])
                 + r03 @ np.array([g.forearm, 0., 0.]))
        result = np.eye(4)
        result[:3, :3] = rotation
        result[:3, 3] = wrist + rotation[:, 2] * g.tool_length
        return result

    def solutions(self, position, rotation, seed=None):
        p, r = np.asarray(position, dtype=float), np.asarray(rotation, dtype=float)
        seed = np.asarray(seed if seed is not None else np.zeros(6), dtype=float)
        if (p.shape != (3,) or r.shape != (3, 3) or seed.shape != (6,)
                or not all(np.all(np.isfinite(a)) for a in (p, r, seed))):
            raise ValueError('Invalid target or seed')
        if not np.allclose(r.T @ r, np.eye(3), atol=1e-7) or not np.isclose(np.linalg.det(r), 1):
            raise ValueError('Target rotation must be in SO(3)')
        g = self.geometry
        w = p - r[:, 2] * g.tool_length - [0, 0, g.base_height]
        radial = np.hypot(w[0], w[1])
        base = np.arctan2(w[1], w[0]) if radial > 1e-9 else np.radians(seed[0])
        answers = []
        for q1, x in ((base, radial), (base + np.pi, -radial)):
            z = -w[2]
            c3 = (x*x + z*z - g.upper_arm**2 - g.forearm**2) / (2*g.upper_arm*g.forearm)
            if abs(c3) > 1 + 1e-9:
                continue
            for q3 in (np.arccos(np.clip(c3, -1, 1)), -np.arccos(np.clip(c3, -1, 1))):
                q2 = np.arctan2(z, x) - np.arctan2(g.forearm*np.sin(q3), g.upper_arm + g.forearm*np.cos(q3))
                wr = (rz(q1) @ ry(q2 + q3)).T @ r
                beta = np.arccos(np.clip(wr[2, 2], -1, 1))
                if abs(np.sin(beta)) > 1e-7:
                    alpha = np.arctan2(wr[1, 2], wr[0, 2])
                    gamma = np.arctan2(wr[2, 1], -wr[2, 0])
                    wrists = [(alpha, beta, gamma), (alpha + np.pi, -beta, gamma + np.pi)]
                else:
                    # Singular wrist: keep q4 close to seed, also try limits.
                    total = np.arctan2(wr[1, 0], wr[0, 0])
                    candidates = [seed[3], 0., *self.limits[3]]
                    for q6 in [seed[5], 0., *self.limits[5]]:
                        candidates.append(np.degrees(total) - q6 if beta < np.pi/2
                                          else np.degrees(total) + q6 - 180.)
                    wrists = []
                    for alpha in np.radians(candidates):
                        gamma = total - alpha if beta < np.pi/2 else alpha + np.pi - total
                        wrists.append((alpha, beta, gamma))
                for wrist in wrists:
                    raw = np.degrees([q1, q2, q3, *wrist])
                    choices = []
                    for value, (lo, hi) in zip(raw, self.limits):
                        k_min = int(np.ceil((lo-value-1e-7)/360))
                        k_max = int(np.floor((hi-value+1e-7)/360))
                        choices.append([value + 360*k for k in range(k_min, k_max+1)])
                    for candidate in product(*choices):
                        candidate = np.clip(candidate, self.limits[:, 0], self.limits[:, 1])
                        t = self.fk(candidate)
                        if (np.allclose(t[:3, 3], p, atol=1e-6)
                                and np.allclose(t[:3, :3], r, atol=1e-6)
                                and not any(np.allclose(candidate, a) for a in answers)):
                            answers.append(candidate)
        # Actual travel, not modulo distance across a physical joint limit.
        return sorted(answers, key=lambda q: float(np.linalg.norm(q-seed)))

    def solve(self, position, rotation, seed=None):
        answers = self.solutions(position, rotation, seed)
        return answers[0].tolist() if answers else None
