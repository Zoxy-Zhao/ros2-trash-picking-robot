"""ROS-independent calibrated plane projection. Units: pixels -> cm."""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class PlaneCalibration:
    width_px: float = 960.
    center_px: float = 200.
    center_x_cm: float = 20.6
    diamond_px: float = 130.
    diamond_radius_cm: float = 4.75


class PlaneProjector:
    def __init__(self, homography, calibration=None):
        self.h = np.asarray(homography, dtype=float)
        self.calibration = calibration or PlaneCalibration()
        if (self.h.shape != (3, 3) or not np.isfinite(self.h).all()
                or np.linalg.matrix_rank(self.h) != 3):
            raise ValueError('Invalid plane homography')
        c = self.calibration
        if not np.isfinite(list(vars(c).values())).all() or min(c.width_px, c.diamond_px, c.diamond_radius_cm) <= 0:
            raise ValueError('Invalid plane calibration')

    def rectify(self, pixel):
        p = np.asarray(pixel, dtype=float)
        if p.shape != (2,) or not np.isfinite(p).all():
            raise ValueError('Expected finite pixel center')
        result = self.h @ np.r_[p, 1.]
        if not np.isfinite(result).all() or abs(result[2]) < 1e-10:
            raise ValueError('Point at projective infinity')
        return result[:2] / result[2]

    def to_base_xy(self, pixel):
        u, v = self.rectify(pixel)
        c = self.calibration
        unit = c.diamond_px / c.diamond_radius_cm
        return (c.center_x_cm + (c.center_px-v)/unit, (u-c.width_px/2)/unit)
