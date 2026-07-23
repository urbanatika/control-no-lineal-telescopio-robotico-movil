"""
Modelo dinámico no lineal del telescopio móvil (base con ruedas + montura
alt-azimutal de 2 GDL) y modelos de perturbación (sísmica, viento, terreno).

Extiende el modelo de eje único del paper base (ec. 1-4) a un manipulador
de 2 grados de libertad: acimut (q1) y elevación (q2), montado sobre una
base móvil que transmite vibración a la estructura.

Convención de estado por eje: x_i = [q_i, qdot_i]
Parámetros físicos reales agrupados en theta = [J, b, mgl] por eje
(idéntico al vector parametrizado theta del paper, ec. 2).
"""
import numpy as np

G_GRAV = 9.81

# ---------------------------------------------------------------------------
# Parámetros físicos "reales" de la planta (desconocidos para el adaptativo)
# ---------------------------------------------------------------------------
class PlantParams:
    """Parámetros reales del telescopio móvil (2 GDL: acimut, elevación)."""

    def __init__(self):
        # Acimut (q1): sin par gravitacional (eje vertical), inercia variable
        # con la elevación por el brazo extendido (acoplamiento geométrico).
        self.J1_0 = 3.2          # kg m^2, inercia base de acimut
        self.J1_coupling = 0.9   # kg m^2, término J1(q2)=J1_0+J1_coupling*cos(q2)^2
        self.b1 = 0.55           # N m s/rad, fricción viscosa acimut

        # Elevación (q2): con par gravitacional (CG del OTA desplazado del eje)
        self.J2 = 1.75           # kg m^2
        self.b2 = 0.40           # N m s/rad
        self.m2 = 3.0            # kg, masa del conjunto brazo+telescopio (escala de banco de laboratorio)
        self.l2 = 0.15           # m, brazo de palanca al CG (contrapeso parcial)

        # Acoplamiento sísmico/vibración de base -> par articular (análogo
        # vectorial de F_s en ec. 3 del paper), uno por eje.
        # NOTA DE ESCALA: los valores numéricos del paper base (Fs, sigma_w)
        # están dimensionados para una antena ALMA de ~100 toneladas
        # (J en el orden de 10^4-10^5 kg m^2). Para el telescopio móvil de
        # banco de laboratorio (J ~ 1-3 kg m^2) se conserva la MISMA
        # estructura matemática (acoplamiento lineal + Gauss-Markov) pero se
        # reescalan las amplitudes proporcionalmente a la inercia/superficie
        # frontal mucho menores de la plataforma pequeña.
        self.Fs1 = 0.55          # N m s^2/m
        self.Fs2 = 0.45          # N m s^2/m

        # Gauss-Markov del viento (ec. 4 del paper), uno por eje
        self.alpha_w1 = 0.6      # 1/s
        self.alpha_w2 = 0.6      # 1/s
        self.sigma_w = 1.8       # N m (std estacionaria objetivo, reescalado -- ver nota arriba)

        # Microvibración de terreno/ruedas (nuevo: naturaleza móvil del robot)
        self.terrain_gain1 = 0.30  # N m / (m/s^2)
        self.terrain_gain2 = 0.25  # N m / (m/s^2)

    def mgl(self):
        return self.m2 * G_GRAV * self.l2

    def theta_true(self):
        """theta = [J1_nom, b1, J2, b2, mgl] -- vector paramétrico real."""
        return np.array([self.J1_0, self.b1, self.J2, self.b2, self.mgl()])


def inertia_azimuth(q2, J1_0, J1_coupling):
    return J1_0 + J1_coupling * np.cos(q2) ** 2


def dJ1_dq2(q2, J1_coupling):
    return -2.0 * J1_coupling * np.cos(q2) * np.sin(q2)


def dynamics_forward(state, tau, tau_dist, params: PlantParams):
    """
    Devuelve qddot = [q1ddot, q2ddot] dado el estado [q1,q1d,q2,q2d],
    el par de control tau=[tau1,tau2] y el par de perturbación
    tau_dist=[taud1,taud2] (sismo+viento+terreno ya sumados).

    Ecuación de Euler-Lagrange por eje (acoplamiento vía J1(q2)):
        J1(q2) q1dd + dJ1/dq2 * q1d*q2d + b1 q1d = tau1 - taud1
        J2      q2dd + 0.5*dJ1/dq2 * q1d^2 + b2 q2d + mgl cos(q2) = tau2 - taud2
    (El término 0.5*dJ1/dq2*q1d^2 es el acoplamiento centrífugo estándar de
    Euler-Lagrange para J1 dependiente de q2; se incluye para consistencia
    energética aunque su magnitud es pequeña frente a J1_0.)
    """
    q1, q1d, q2, q2d = state
    J1 = inertia_azimuth(q2, params.J1_0, params.J1_coupling)
    dJ1 = dJ1_dq2(q2, params.J1_coupling)

    coriolis1 = dJ1 * q1d * q2d
    q1dd = (tau[0] - tau_dist[0] - coriolis1 - params.b1 * q1d) / J1

    centrifugal2 = 0.5 * dJ1 * q1d ** 2
    grav2 = params.mgl() * np.cos(q2)
    q2dd = (tau[1] - tau_dist[1] - centrifugal2 - params.b2 * q2d - grav2) / params.J2

    return np.array([q1dd, q2dd])


def regressor_azimuth(q1dd_r, q1d_actual):
    """
    Matriz de regresión Y1 tal que Y1 @ [J1_0, b1] ≈ J1(q2) q1dd_r + b1 q1d
    (para el eje de acimut, ley adaptativa por eje, ver ec. 2 del paper).
    q1dd_r es la aceleración de referencia virtual (ec. 7 del paper);
    q1d_actual es la velocidad MEDIDA/ESTIMADA real (el término de fricción
    depende del movimiento real, no de la referencia), igual que en
    Y(q,qdot,qdot_r,qddot_r)=[qddot_r, qdot, sin(q)] del paper.
    Se usa el nominal J1(q2)=J1_0 (ignorando J1_coupling en la
    parametrización lineal -- término pequeño absorbido por el término
    robusto -Kd*s, práctica estándar de control adaptativo decentralizado).
    """
    return np.array([q1dd_r, q1d_actual])


def regressor_elevation(q2dd_r, q2d_actual, q2):
    """
    Matriz de regresión Y2 tal que Y2 @ [J2, b2, mgl] ≈ J2 q2dd_r + b2 q2d + mgl cos(q2)
    (idéntica en estructura a Y(q,qdot,qdot_r,qddot_r)=[qddot_r, qdot, sin(q)]
    del paper, ec. 2, adaptada a cos(q2) por la convención de elevación).
    """
    return np.array([q2dd_r, q2d_actual, np.cos(q2)])


# ---------------------------------------------------------------------------
# Perturbaciones: sismo multi-evento + viento Gauss-Markov + terreno
# ---------------------------------------------------------------------------
def seismic_acceleration(t, rng):
    """
    Señal sintética de aceleración de suelo x_gddot(t) [m/s^2], emulando el
    mismo perfil multi-evento de 60 s del paper base:
      Fase 1 (0-5s):   ruido basal del desierto (<=0.05 m/s^2)
      Fase 2 (5-25s):  Coquimbo 2015 (Mw8.4) - alta frecuencia, caótico, transitorio
      Fase 3 (25-45s): Chiloé 2016 (Mw7.6) - baja frecuencia, oscilatorio lento, gran amplitud
      Fase 4 (45-60s): Enjambre Melipilla 2017 (Mw6.9) - ráfagas impulsivas cortas
    No son registros reales del CSN; son señales sintéticas con las
    características espectrales/de envolvente descritas en el paper,
    generadas por combinación de sinusoides moduladas + ruido filtrado.
    """
    if t < 5.0:
        return rng.normal(0, 0.02)

    if t < 25.0:  # Coquimbo: alta frecuencia (3-10 Hz), envolvente rápida
        tau = t - 5.0
        envelope = 3.6 * np.exp(-((tau - 4.0) ** 2) / (2 * 3.0 ** 2)) + 0.6
        hf = (np.sin(2 * np.pi * 6.3 * tau) + 0.6 * np.sin(2 * np.pi * 9.1 * tau + 0.4)
              + 0.4 * rng.normal())
        return envelope * hf

    if t < 45.0:  # Chiloé: baja frecuencia (0.3-1 Hz), gran amplitud, oscilatorio lento
        tau = t - 25.0
        envelope = 4.4 * np.exp(-((tau - 7.0) ** 2) / (2 * 6.0 ** 2)) + 0.5
        lf = (np.sin(2 * np.pi * 0.55 * tau) + 0.5 * np.sin(2 * np.pi * 0.85 * tau + 0.9)
              + 0.25 * rng.normal())
        return envelope * lf

    # Melipilla: enjambre de ráfagas impulsivas cortas (45-60s)
    tau = t - 45.0
    bursts = 0.0
    for k in range(6):
        t0 = 1.0 + k * 2.3
        bursts += 2.8 * np.exp(-((tau - t0) ** 2) / (2 * 0.25 ** 2)) * np.sin(2 * np.pi * 7.5 * (tau - t0))
    return bursts + 0.3 * rng.normal()


def terrain_acceleration(t, rng, prev, dt):
    """
    Microvibración de terreno/ruedas del robot móvil: ruido de banda
    limitada 5-50 Hz, ausente en el paper original (antena fija).
    Filtro pasa-banda simple de 1er orden aplicado a ruido blanco.
    """
    white = rng.normal(0, 0.35)
    f_lo, f_hi = 5.0, 50.0
    a = np.exp(-2 * np.pi * f_lo * dt)
    hp = white - prev.get("hp_prev", 0.0) + a * prev.get("hp_state", 0.0)
    prev["hp_state"] = hp
    prev["hp_prev"] = white
    b = np.exp(-2 * np.pi * f_hi * dt)
    lp = (1 - b) * hp + b * prev.get("lp_state", 0.0)
    prev["lp_state"] = lp
    return lp


class WindGaussMarkov:
    """Proceso de Gauss-Markov de 1er orden por eje (ec. 4 del paper)."""

    def __init__(self, alpha, sigma_target, dt, rng):
        self.alpha = alpha
        self.dt = dt
        self.rng = rng
        # varianza estacionaria de un GM continuo: sigma^2 = Q/(2*alpha)
        self.Q = 2 * alpha * sigma_target ** 2
        self.tau_v = 0.0

    def step(self):
        w = self.rng.normal(0, np.sqrt(self.Q / self.dt))
        self.tau_v += (-self.alpha * self.tau_v + w) * self.dt
        return self.tau_v


def build_disturbance_generators(params: PlantParams, dt, seed):
    rng = np.random.default_rng(seed)
    wind1 = WindGaussMarkov(params.alpha_w1, params.sigma_w, dt, rng)
    wind2 = WindGaussMarkov(params.alpha_w2, params.sigma_w, dt, rng)
    terrain_state = {}
    return rng, wind1, wind2, terrain_state


def disturbance_torque(t, dt, rng, wind1, wind2, terrain_state, params: PlantParams):
    """
    Devuelve (tau_dist[2], xgddot, tau_wind[2], terrain_acc) para logging
    del EKF/ground truth.

    tau_dist es el término que dynamics_forward RESTA íntegro de tau (ver
    ec. de qdd). Para que esto reproduzca la convención de signos del paper
    (planta: J*qdd + b*qd + mgl*sin(q) = tau - tau_s + tau_v, ec. 1), el
    viento debe entrar con signo NEGATIVO dentro de tau_dist, de forma que
    "tau - tau_dist" = "tau - tau_s + tau_v - tau_terrain". El feedforward
    de la ley de control (+Fs*xgddot - tau_v_hat, ec. 11) está diseñado
    exactamente para cancelar esta combinación.
    """
    xgddot = seismic_acceleration(t, rng)
    terr = terrain_acceleration(t, rng, terrain_state, dt)
    tv1 = wind1.step()
    tv2 = wind2.step()

    taud1 = params.Fs1 * xgddot - tv1 + params.terrain_gain1 * terr
    taud2 = params.Fs2 * xgddot - tv2 + params.terrain_gain2 * terr
    return np.array([taud1, taud2]), xgddot, np.array([tv1, tv2]), terr
