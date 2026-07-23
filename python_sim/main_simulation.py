"""
Simulación comparativa de las 4 leyes de control sobre el modelo del
telescopio móvil de 2 GDL, bajo perturbación multi-evento idéntica
(mismo seed) para cada ley -> comparación pareada justa.

Réplica en Python (numpy/scipy) del script MATLAB equivalente
matlab/main_simulation.m, ejecutada aquí porque el MATLAB local no
tiene binario funcional (ver plan). Resultados y figuras -> ../figures/
"""
import time
import numpy as np

from model import (PlantParams, dynamics_forward, build_disturbance_generators,
                    disturbance_torque, G_GRAV)
from ekf import AxisEKF
from controllers import (AdaptiveLyapunovController, SMCController,
                          MPCController, NeuroFuzzyController)

DT = 1e-3
T_FINAL = 60.0
N_STEPS = int(T_FINAL / DT)
SEED = 20250716

SIGMA_ENC = 0.002   # rad
SIGMA_GYRO = 0.01   # rad/s

Q1_0 = np.deg2rad(45.0)
Q2_0 = np.deg2rad(30.0)
OMEGA1 = 7.292e-5   # rad/s, tasa sideral (acimut)
OMEGA2 = 2.0e-5     # rad/s, deriva de elevación


class Refs:
    __slots__ = ("q1", "q1d", "q1dd", "q2", "q2d", "q2dd")

    def __init__(self, q1, q1d, q1dd, q2, q2d, q2dd):
        self.q1, self.q1d, self.q1dd = q1, q1d, q1dd
        self.q2, self.q2d, self.q2dd = q2, q2d, q2dd


class FF:
    __slots__ = ("seismic1", "seismic2", "wind1", "wind2")

    def __init__(self, seismic1, seismic2, wind1, wind2):
        self.seismic1, self.seismic2 = seismic1, seismic2
        self.wind1, self.wind2 = wind1, wind2


def reference(t):
    return Refs(Q1_0 + OMEGA1 * t, OMEGA1, 0.0,
                Q2_0 + OMEGA2 * t, OMEGA2, 0.0)


def phase_of(t):
    if t < 5.0:
        return "Basal"
    if t < 25.0:
        return "Coquimbo"
    if t < 45.0:
        return "Chiloe"
    return "Melipilla"


def run_one(controller_ctor, params: PlantParams, label):
    rng, wind1, wind2, terrain_state = build_disturbance_generators(params, DT, SEED)

    ekf1 = AxisEKF(DT, params.J1_0, params.b1, 0.0, params.alpha_w1, False,
                   SIGMA_ENC, SIGMA_GYRO)
    ekf2 = AxisEKF(DT, params.J2, params.b2, params.mgl(), params.alpha_w2, True,
                   SIGMA_ENC, SIGMA_GYRO)

    r0 = reference(0.0)
    state = np.array([r0.q1 + np.deg2rad(0.15), 0.0, r0.q2 - np.deg2rad(0.10), 0.0])
    ekf1.x[:] = [state[0], 0.0, 0.0]
    ekf2.x[:] = [state[2], 0.0, 0.0]

    controller = controller_ctor(params, DT)
    is_mpc = isinstance(controller, MPCController)

    log = {
        "t": np.zeros(N_STEPS), "err1": np.zeros(N_STEPS), "err2": np.zeros(N_STEPS),
        "err_total_arcsec": np.zeros(N_STEPS), "tau1": np.zeros(N_STEPS), "tau2": np.zeros(N_STEPS),
        "q1": np.zeros(N_STEPS), "q2": np.zeros(N_STEPS),
        "tv1_true": np.zeros(N_STEPS), "tv1_hat": np.zeros(N_STEPS),
        "tv2_true": np.zeros(N_STEPS), "tv2_hat": np.zeros(N_STEPS),
        "s1": np.zeros(N_STEPS), "s2": np.zeros(N_STEPS),
        "theta1": [], "theta2": [],
    }
    t_compute_total = 0.0

    for k in range(N_STEPS):
        t = k * DT
        refs = reference(t)

        tau_dist, xgddot, tau_wind_true, terr = disturbance_torque(
            t, DT, rng, wind1, wind2, terrain_state, params)

        z1 = np.array([state[0] + rng.normal(0, SIGMA_ENC), state[1] + rng.normal(0, SIGMA_GYRO)])
        z2 = np.array([state[2] + rng.normal(0, SIGMA_ENC), state[3] + rng.normal(0, SIGMA_GYRO)])

        ff = FF(params.Fs1 * xgddot, params.Fs2 * xgddot, ekf1.x[2], ekf2.x[2])

        t0 = time.perf_counter()
        if is_mpc:
            tau, diag = controller.step(t, ekf1.x[[0, 1]].tolist() + ekf2.x[[0, 1]].tolist(),
                                         reference, ff, DT)
        else:
            state_hat = (ekf1.x[0], ekf1.x[1], ekf2.x[0], ekf2.x[1])
            tau, diag = controller.step(state_hat, refs, ff, DT)
        t_compute_total += time.perf_counter() - t0

        qdd = dynamics_forward(state, tau, tau_dist, params)
        state = state + DT * np.array([state[1], qdd[0], state[3], qdd[1]])

        ekf1.predict(tau[0], params.Fs1 * xgddot)
        ekf1.update(z1)
        ekf2.predict(tau[1], params.Fs2 * xgddot)
        ekf2.update(z2)

        e1 = state[0] - refs.q1
        e2 = state[2] - refs.q2
        log["t"][k] = t
        log["err1"][k] = e1
        log["err2"][k] = e2
        log["err_total_arcsec"][k] = np.hypot(e1, e2) * 206264.806
        log["tau1"][k] = tau[0]
        log["tau2"][k] = tau[1]
        log["q1"][k] = state[0]
        log["q2"][k] = state[2]
        log["tv1_true"][k] = tau_wind_true[0]
        log["tv1_hat"][k] = ekf1.x[2]
        log["tv2_true"][k] = tau_wind_true[1]
        log["tv2_hat"][k] = ekf2.x[2]
        if "theta1" in diag:
            log["theta1"].append(diag["theta1"])
            log["theta2"].append(diag["theta2"])
        if "s1" in diag:
            log["s1"][k] = diag["s1"]
            log["s2"][k] = diag["s2"]

    log["theta1"] = np.array(log["theta1"]) if log["theta1"] else None
    log["theta2"] = np.array(log["theta2"]) if log["theta2"] else None
    log["compute_time_per_step_ms"] = 1000.0 * t_compute_total / N_STEPS
    log["label"] = label
    print(f"  [{label}] listo. tiempo cómputo promedio/paso = {log['compute_time_per_step_ms']:.4f} ms")
    return log


def compute_metrics(log):
    t = log["t"]
    phases = {"Basal": (0, 5), "Coquimbo": (5, 25), "Chiloe": (25, 45), "Melipilla": (45, 60)}
    err = log["err_total_arcsec"]
    tau1, tau2 = log["tau1"], log["tau2"]
    metrics = {"label": log["label"], "compute_ms": log["compute_time_per_step_ms"]}
    for name, (t0, t1) in phases.items():
        mask = (t >= t0) & (t < t1)
        metrics[f"rmse_{name}"] = float(np.sqrt(np.mean(err[mask] ** 2)))
        metrics[f"max_{name}"] = float(np.max(err[mask]))
    metrics["rmse_total"] = float(np.sqrt(np.mean(err ** 2)))
    metrics["max_total"] = float(np.max(err))
    metrics["effort"] = float(np.sum(tau1 ** 2 + tau2 ** 2) * DT)
    metrics["chattering"] = float(np.sum(np.abs(np.diff(tau1))) + np.sum(np.abs(np.diff(tau2))))
    return metrics


def run_all():
    params = PlantParams()
    controllers = [
        (AdaptiveLyapunovController, "Adaptativo-Lyapunov+EKF"),
        (SMCController, "SMC (Super-Twisting)"),
        (MPCController, "MPC"),
        (NeuroFuzzyController, "Neuro-Difuso Adaptativo"),
    ]
    logs, metrics_all = {}, []
    print(f"Simulando {N_STEPS} pasos (60 s @ 1 kHz) x 4 leyes de control...")
    for ctor, label in controllers:
        print(f"-> Corriendo {label} ...")
        log = run_one(ctor, params, label)
        logs[label] = log
        metrics_all.append(compute_metrics(log))
    return logs, metrics_all, params


if __name__ == "__main__":
    logs, metrics_all, params = run_all()
    for m in metrics_all:
        print(m)
