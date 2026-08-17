"""
Las 4 leyes de control comparadas, todas de diseño DESCENTRALIZADO
(un lazo independiente por eje: acimut sin gravedad, elevación con
gravedad), todas alimentadas por:
  - Feedforward sísmico directo Fs*xgddot(t) (sismómetro de fundación,
    igual que ec. 11 del paper base).
  - Estimación de viento tau_v_hat proveniente del EKF (ec. 13-19).
  - Estado limpio (q_hat, qdot_hat) proveniente del EKF.

La perturbación de MICROVIBRACIÓN DE TERRENO (nueva, propia del robot
móvil) NO se compensa por feedforward para ninguna ley: es el residuo no
modelado contra el cual se mide la robustez comparativa de las 4
estrategias, junto con un error de parámetros nominales intencional
(offset respecto a los valores reales, "parcialmente desconocidos").

Cada ley se implementa como una clase con método .compute(...) -> tau,
por eje, y un wrapper de 2 ejes al final de cada clase.
"""
import numpy as np

from model import regressor_azimuth, regressor_elevation, G_GRAV


TAU_MAX = 40.0  # N m, límite de par del actuador (restricción dura para MPC)


# ---------------------------------------------------------------------------
# A. Control Adaptativo de Lyapunov (extensión vectorial de ec. 5-12 paper)
# ---------------------------------------------------------------------------
class AdaptiveAxis:
    def __init__(self, has_gravity, theta0, Gamma_diag, lam, Kd):
        self.has_gravity = has_gravity
        self.theta_hat = np.array(theta0, dtype=float)
        self.Gamma = np.diag(Gamma_diag)
        self.lam = lam
        self.Kd = Kd

    def compute(self, q, qd, q_des, qd_des, qdd_des, ff_seismic, ff_wind, dt):
        e = q - q_des
        ed = qd - qd_des
        s = ed + self.lam * e
        qd_r = qd_des - self.lam * e
        qdd_r = qdd_des - self.lam * ed

        if self.has_gravity:
            Y = regressor_elevation(qdd_r, qd, q)
        else:
            Y = regressor_azimuth(qdd_r, qd)

        tau = Y @ self.theta_hat - self.Kd * s + ff_seismic - ff_wind
        tau = np.clip(tau, -TAU_MAX, TAU_MAX)

        # Ley de adaptación (ec. 12): thetadot = -Gamma * Y^T * s
        self.theta_hat = self.theta_hat - self.Gamma @ Y * s * dt
        return tau, s


class AdaptiveLyapunovController:
    name = "Adaptativo-Lyapunov+EKF"

    def __init__(self, params, dt):
        # theta0 con error intencional (~30%) respecto al valor real,
        # representando incertidumbre paramétrica inicial.
        theta1_0 = [params.J1_0 * 1.3, params.b1 * 0.7]
        theta2_0 = [params.J2 * 0.7, params.b2 * 1.3, params.mgl() * 1.25]
        self.az = AdaptiveAxis(False, theta1_0, [1.2, 0.3], lam=10.0, Kd=22.0)
        self.el = AdaptiveAxis(True, theta2_0, [1.0, 0.25, 6.0], lam=10.0, Kd=22.0)

    def step(self, state, refs, ff, dt):
        q1, q1d, q2, q2d = state
        tau1, s1 = self.az.compute(q1, q1d, refs.q1, refs.q1d, refs.q1dd,
                                    ff.seismic1, ff.wind1, dt)
        tau2, s2 = self.el.compute(q2, q2d, refs.q2, refs.q2d, refs.q2dd,
                                    ff.seismic2, ff.wind2, dt)
        diag = {"theta1": self.az.theta_hat.copy(), "theta2": self.el.theta_hat.copy(),
                "s1": s1, "s2": s2}
        return np.array([tau1, tau2]), diag


# ---------------------------------------------------------------------------
# B. Control por Modos Deslizantes de 2do orden (Super-Twisting)
# ---------------------------------------------------------------------------
class SMCAxis:
    def __init__(self, has_gravity, theta_nom, lam, k1, k2):
        self.has_gravity = has_gravity
        self.theta_nom = np.array(theta_nom, dtype=float)
        self.lam = lam
        self.k1 = k1
        self.k2 = k2
        self.w = 0.0

    def compute(self, q, qd, q_des, qd_des, qdd_des, ff_seismic, ff_wind, dt):
        e = q - q_des
        ed = qd - qd_des
        s = ed + self.lam * e
        qd_r = qd_des - self.lam * e
        qdd_r = qdd_des - self.lam * ed

        if self.has_gravity:
            Y = regressor_elevation(qdd_r, qd, q)
        else:
            Y = regressor_azimuth(qdd_r, qd)

        tau_eq = Y @ self.theta_nom + ff_seismic - ff_wind

        # Super-Twisting (Moreno & Osorio 2012): continuo, reduce chattering
        u1 = -self.k1 * np.sqrt(np.abs(s)) * np.sign(s) + self.w
        self.w += (-self.k2 * np.sign(s)) * dt

        tau = np.clip(tau_eq + u1, -TAU_MAX, TAU_MAX)
        return tau, s


class SMCController:
    name = "SMC (Super-Twisting)"

    def __init__(self, params, dt):
        theta1_nom = [params.J1_0 * 1.3, params.b1 * 0.7]
        theta2_nom = [params.J2 * 0.7, params.b2 * 1.3, params.mgl() * 1.25]
        self.az = SMCAxis(False, theta1_nom, lam=6.0, k1=6.0, k2=18.0)
        self.el = SMCAxis(True, theta2_nom, lam=6.0, k1=6.0, k2=18.0)

    def step(self, state, refs, ff, dt):
        q1, q1d, q2, q2d = state
        tau1, s1 = self.az.compute(q1, q1d, refs.q1, refs.q1d, refs.q1dd,
                                    ff.seismic1, ff.wind1, dt)
        tau2, s2 = self.el.compute(q2, q2d, refs.q2, refs.q2d, refs.q2dd,
                                    ff.seismic2, ff.wind2, dt)
        return np.array([tau1, tau2]), {"s1": s1, "s2": s2}


# ---------------------------------------------------------------------------
# C. MPC lineal de horizonte deslizante, formulación QP en lote (batch)
#    resuelta en forma CERRADA (mínimos cuadrados regularizados + recorte
#    a los límites de par). El modelo de predicción se linealiza en torno
#    a los parámetros nominales (J,b y, para elevación, la gravedad
#    linealizada como offset constante evaluado en el q actual, válido
#    para el horizonte corto de ~50 ms usado aquí). Las matrices de
#    predicción (Phi, Gamma) y la ganancia batch K_mpc son constantes y
#    se precalculan una sola vez en el constructor -> evaluación por paso
#    es solo un par de productos matriz-vector (recalculado a 200 Hz).
# ---------------------------------------------------------------------------
class MPCAxis:
    def __init__(self, has_gravity, J_nom, b_nom, mgl_nom, N, dt_mpc, Qw, Rw, rho_rate=5.0):
        self.has_gravity = has_gravity
        self.J = J_nom
        self.b = b_nom
        self.mgl = mgl_nom
        self.N = N
        self.dt_mpc = dt_mpc
        self.tau_hold = 0.0

        n = 2  # estado [e_q, e_qdot] en variables absolutas [q, qdot]
        A = np.array([[0.0, 1.0], [0.0, -b_nom / J_nom]])
        B = np.array([[0.0], [1.0 / J_nom]])
        self.Ad = np.eye(n) + A * dt_mpc
        self.Bd = B * dt_mpc

        Apows = [np.eye(n)]
        for _ in range(N):
            Apows.append(Apows[-1] @ self.Ad)

        Phi = np.zeros((N * n, n))
        Gamma = np.zeros((N * n, N))
        for k in range(N):
            Phi[k * n:(k + 1) * n, :] = Apows[k + 1]
            for j in range(k + 1):
                Gamma[k * n:(k + 1) * n, j] = (Apows[k - j] @ self.Bd).flatten()

        Qbar = np.kron(np.eye(N), np.diag(Qw))
        D = np.eye(N) - np.eye(N, k=-1)
        D[0, 0] = 0.0  # no penaliza el salto respecto al par anterior aplicado
        Rbar = np.eye(N) * Rw + rho_rate * Rw * (D.T @ D)

        H = Gamma.T @ Qbar @ Gamma + Rbar
        self.K_mpc = np.linalg.solve(H, Gamma.T @ Qbar)  # (N x N n)
        self.Phi = Phi
        self.GammaOnes = Gamma @ np.ones(N)

    def recompute(self, q, qd, q_ref_seq, qd_ref_seq, bias):
        grav = self.mgl * np.cos(q) if self.has_gravity else 0.0
        c = -(bias + grav) / 1.0  # offset ya está en unidades de aceleración*J via Bd; ver nota abajo
        # c representa el "par equivalente" que produce la misma aceleración
        # que -(bias+grav) al pasar por Bd=[0;dt/J], consistente con
        # qdd = (u - bias - b*qd - grav)/J  =>  aporte extra = Bd * c con c=-(bias+grav)
        x0 = np.array([q, qd])
        Xref = np.empty(self.N * 2)
        Xref[0::2] = q_ref_seq
        Xref[1::2] = qd_ref_seq

        d = self.Phi @ x0 + self.GammaOnes * c - Xref
        U = -self.K_mpc @ d
        U = np.clip(U, -TAU_MAX, TAU_MAX)
        self.tau_hold = float(U[0])
        return self.tau_hold


class MPCController:
    name = "MPC"

    def __init__(self, params, dt, N=15, dt_mpc=0.01):
        # Parámetros nominales con el mismo error intencional que las demás leyes
        self.az = MPCAxis(False, params.J1_0 * 1.3, params.b1 * 0.7, 0.0,
                           N, dt_mpc, Qw=(4000.0, 40.0), Rw=5e-4, rho_rate=2.0)
        self.el = MPCAxis(True, params.J2 * 0.7, params.b2 * 1.3, params.mgl() * 1.25,
                           N, dt_mpc, Qw=(4000.0, 40.0), Rw=5e-4, rho_rate=2.0)
        self.dt_mpc = dt_mpc
        self.t_last = -1.0

    def step(self, t, state, ref_fn, ff, dt):
        q1, q1d, q2, q2d = state
        if t - self.t_last >= self.dt_mpc - 1e-9:
            self.t_last = t
            tgrid = t + np.arange(1, self.az.N + 1) * self.dt_mpc
            refs_seq = [ref_fn(tt) for tt in tgrid]
            q1_ref = np.array([r.q1 for r in refs_seq])
            qd1_ref = np.array([r.q1d for r in refs_seq])
            q2_ref = np.array([r.q2 for r in refs_seq])
            qd2_ref = np.array([r.q2d for r in refs_seq])

            bias1 = ff.seismic1 - ff.wind1
            bias2 = ff.seismic2 - ff.wind2
            self.az.recompute(q1, q1d, q1_ref, qd1_ref, bias1)
            self.el.recompute(q2, q2d, q2_ref, qd2_ref, bias2)

        return np.array([self.az.tau_hold, self.el.tau_hold]), {}


# ---------------------------------------------------------------------------
# D. Control Neuro-Difuso Adaptativo (base gaussiana, ley de Lyapunov)
# ---------------------------------------------------------------------------
class FuzzyAxis:
    def __init__(self, has_gravity, theta_nom, lam, Kd, n_mf=5, gamma=40.0, sigma_mod=0.01):
        self.has_gravity = has_gravity
        self.theta_nom = np.array(theta_nom, dtype=float)
        self.lam = lam
        self.Kd = Kd
        centers_e = np.linspace(-0.05, 0.05, n_mf)
        centers_ed = np.linspace(-0.3, 0.3, n_mf)
        Ce, Ced = np.meshgrid(centers_e, centers_ed)
        self.centers = np.stack([Ce.ravel(), Ced.ravel()], axis=1)
        self.width = 0.04
        self.theta_fuzzy = np.zeros(self.centers.shape[0])
        self.gamma = gamma
        self.sigma_mod = sigma_mod

    def basis(self, e, ed):
        d2 = (self.centers[:, 0] - e) ** 2 + (self.centers[:, 1] - ed) ** 2
        phi = np.exp(-d2 / (2 * self.width ** 2))
        s = phi.sum()
        return phi / s if s > 1e-9 else phi

    def compute(self, q, qd, q_des, qd_des, qdd_des, ff_seismic, ff_wind, dt):
        e = q - q_des
        ed = qd - qd_des
        s = ed + self.lam * e
        qd_r = qd_des - self.lam * e
        qdd_r = qdd_des - self.lam * ed

        if self.has_gravity:
            Y = regressor_elevation(qdd_r, qd, q)
        else:
            Y = regressor_azimuth(qdd_r, qd)

        xi = self.basis(e, ed)
        delta_hat = self.theta_fuzzy @ xi

        tau_base = Y @ self.theta_nom - self.Kd * s + ff_seismic - ff_wind
        tau = np.clip(tau_base - delta_hat, -TAU_MAX, TAU_MAX)

        # Ley de adaptación difusa (Wang 1993): thetadot = Gamma*xi*s - sigma*theta
        self.theta_fuzzy += (self.gamma * xi * s - self.sigma_mod * self.theta_fuzzy) * dt
        return tau, s


class NeuroFuzzyController:
    name = "Neuro-Difuso Adaptativo"

    def __init__(self, params, dt):
        theta1_nom = [params.J1_0 * 1.3, params.b1 * 0.7]
        theta2_nom = [params.J2 * 0.7, params.b2 * 1.3, params.mgl() * 1.25]
        self.az = FuzzyAxis(False, theta1_nom, lam=10.0, Kd=20.0, gamma=80.0)
        self.el = FuzzyAxis(True, theta2_nom, lam=10.0, Kd=20.0, gamma=80.0)

    def step(self, state, refs, ff, dt):
        q1, q1d, q2, q2d = state
        tau1, s1 = self.az.compute(q1, q1d, refs.q1, refs.q1d, refs.q1dd,
                                    ff.seismic1, ff.wind1, dt)
        tau2, s2 = self.el.compute(q2, q2d, refs.q2, refs.q2d, refs.q2dd,
                                    ff.seismic2, ff.wind2, dt)
        return np.array([tau1, tau2]), {"s1": s1, "s2": s2}
