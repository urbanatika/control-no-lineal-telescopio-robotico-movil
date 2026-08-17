"""
Filtro de Kalman Extendido (EKF), extensión directa por-eje de las
ecuaciones 13-19 del paper base. Estado por eje: x = [q, qdot, tau_v].
Se instancian dos EKF independientes (acimut y elevación) -- diseño
decentralizado, consistente con el resto de las leyes de control.
"""
import numpy as np


class AxisEKF:
    def __init__(self, dt, J_nom, b_nom, mgl_nom, alpha_w, has_gravity,
                 sigma_encoder, sigma_gyro, q_process=None):
        self.dt = dt
        self.J = J_nom
        self.b = b_nom
        self.mgl = mgl_nom
        self.alpha = alpha_w
        self.has_gravity = has_gravity

        self.x = np.zeros(3)  # [q, qdot, tau_v]
        self.P = np.diag([1e-3, 1e-2, 25.0])

        self.Q = np.diag(q_process if q_process is not None else [1e-8, 1e-5, 4.0])
        self.R = np.diag([sigma_encoder ** 2, sigma_gyro ** 2])
        self.H = np.array([[1.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0]])

    def f(self, x, tau, tau_seismic_wind_free):
        """Dinámica no lineal continua x_dot = f(x, tau, disturbancias conocidas)."""
        q, qd, tv = x
        grav = self.mgl * np.cos(q) if self.has_gravity else 0.0
        qdd = (tau - tau_seismic_wind_free - self.b * qd - grav + tv) / self.J
        tvd = -self.alpha * tv
        return np.array([qd, qdd, tvd])

    def jacobian(self, x):
        q, qd, tv = x
        F = np.zeros((3, 3))
        F[0, 1] = 1.0
        if self.has_gravity:
            F[1, 0] = self.mgl * np.sin(q) / self.J
        F[1, 1] = -self.b / self.J
        F[1, 2] = 1.0 / self.J
        F[2, 2] = -self.alpha
        return F

    def predict(self, tau, tau_seismic_component):
        xdot = self.f(self.x, tau, tau_seismic_component)
        self.x = self.x + xdot * self.dt
        Phi = np.eye(3) + self.jacobian(self.x) * self.dt
        self.P = Phi @ self.P @ Phi.T + self.Q

    def update(self, z_meas):
        y = z_meas - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(3) - K @ self.H) @ self.P
