import numpy as np
from . import quaternion


class ExtendedKalmanFilter:
    def __init__(self, x0, P0_value, dt, g):
        self.x = np.array([x0], dtype=float).T
        self.P = np.eye(len(x0), dtype=float) * P0_value
        self.dt = dt
        self.g = g

    def predict(self, u, f, F, Q, ctrl_vars):
        calc_f = f(*self.x[:, 0], *u, self.g, self.dt)
        calc_F = F(*self.x[:, 0], *u, self.g, self.dt)
        calc_Q = Q(*self.x[:, 0], *u, *ctrl_vars, self.g, self.dt)

        self.x = calc_f
        self.P = calc_F @ self.P @ calc_F.T + calc_Q

        #self._normalize_quat()
        self._force_cov_symmetry()

    def correct(self, z, h, H, R, meas_vars):
        z = np.array([z]).T

        calc_h = h(*self.x[:, 0])
        calc_H = H(*self.x[:, 0])
        calc_R = R(*meas_vars)

        PH_ = self.P @ calc_H.T
        K = PH_ @ np.linalg.inv(calc_H @ PH_ + calc_R)
        self.x = self.x + K @ (z - calc_h)
        I_KH = np.eye(calc_H.shape[1]) - K @ calc_H
        self.P = I_KH @ self.P @ I_KH.T + K @ calc_R @ K.T

        #self._normalize_quat()
        self._force_cov_symmetry()

    def _normalize_quat(self):
        self.x[0:4, 0] = quaternion.quat_normalize(self.x[0:4, 0])

    def _force_cov_symmetry(self):
        for i in range(len(self.P)):
            for j in range(i):
                tmp = (self.P[i, j] + self.P[j, i]) / 2
                self.P[i, j] = tmp
                self.P[j, i] = tmp
