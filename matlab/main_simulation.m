% MAIN_SIMULATION Simulación comparativa de las 4 leyes de control sobre
% el telescopio móvil de 2 GDL (acimut, elevación), bajo perturbación
% multi-evento idéntica (mismo seed) para cada ley -> comparación pareada.
%
% Réplica exacta (misma estructura, parámetros y ganancias) del script
% python_sim/main_simulation.py, verificado y ejecutado en Python porque
% el MATLAB local de este equipo no tenía binario funcional al momento de
% escribir este proyecto. Ejecutar este .m cuando se disponga de una
% licencia de MATLAB activa (ver informe, Sección "Reproducibilidad").
%
% Salidas: struct `logs` (uno por controlador) y `metrics` (tabla resumen),
% además de todas las figuras comparativas en ../figures/.

clear; clc; close all;
% (el seed se fija dentro de run_one, una vez por controlador -- ver abajo)

DT = 1e-3;
T_FINAL = 60.0;
N_STEPS = round(T_FINAL / DT);

SIGMA_ENC = 0.002;   % rad
SIGMA_GYRO = 0.01;   % rad/s

Q1_0 = deg2rad(45.0);
Q2_0 = deg2rad(30.0);
OMEGA1 = 7.292e-5;   % rad/s (tasa sideral, acimut)
OMEGA2 = 2.0e-5;     % rad/s (deriva lenta, elevación)

params = plant_model();

controller_labels = {'Adaptativo-Lyapunov+EKF', 'SMC (Super-Twisting)', 'MPC', 'Neuro-Difuso Adaptativo'};
logs = struct();
metrics = struct([]);

for c = 1:4
    label = controller_labels{c};
    fprintf('-> Corriendo %s ...\n', label);
    log = run_one(c, params, DT, N_STEPS, T_FINAL, SIGMA_ENC, SIGMA_GYRO, ...
                  Q1_0, Q2_0, OMEGA1, OMEGA2, label);
    field = matlab.lang.makeValidName(label);
    logs.(field) = log;
    metrics(c).label = label;
    metrics(c) = compute_metrics(log, DT);
end

save('sim_results_matlab.mat', 'logs', 'metrics');
disp(struct2table(metrics));

% ------------------------------------------------------------------
function log = run_one(controller_type, params, dt, n_steps, t_final, ...
                        sigma_enc, sigma_gyro, q1_0, q2_0, omega1, omega2, label)

    rng(20250716, 'twister');   % reinicia el stream por controlador: comparación pareada
    wind_state.tv1 = 0.0; wind_state.tv2 = 0.0;
    terrain_state.hp_state = 0.0; terrain_state.hp_prev = 0.0; terrain_state.lp_state = 0.0;

    ekf1 = AxisEKF(dt, params.J1_0, params.b1, 0.0, params.alpha_w1, false, sigma_enc, sigma_gyro);
    ekf2 = AxisEKF(dt, params.J2, params.b2, params.mgl, params.alpha_w2, true, sigma_enc, sigma_gyro);

    state = [q1_0 + deg2rad(0.15), 0.0, q2_0 - deg2rad(0.10), 0.0];
    ekf1.x = [state(1); 0.0; 0.0];
    ekf2.x = [state(3); 0.0; 0.0];

    theta1_0 = [params.J1_0 * 1.3, params.b1 * 0.7];
    theta2_0 = [params.J2 * 0.7, params.b2 * 1.3, params.mgl * 1.25];

    switch controller_type
        case 1  % Adaptativo-Lyapunov
            az = AdaptiveAxis(false, theta1_0, [1.2, 0.3], 10.0, 22.0);
            el = AdaptiveAxis(true, theta2_0, [1.0, 0.25, 6.0], 10.0, 22.0);
        case 2  % SMC
            az = SMCAxis(false, theta1_0, 6.0, 6.0, 18.0);
            el = SMCAxis(true, theta2_0, 6.0, 6.0, 18.0);
        case 3  % MPC
            az = MPCAxis(false, params.J1_0*1.3, params.b1*0.7, 0.0, 15, 0.01, [4000.0, 40.0], 5e-4, 2.0);
            el = MPCAxis(true, params.J2*0.7, params.b2*1.3, params.mgl*1.25, 15, 0.01, [4000.0, 40.0], 5e-4, 2.0);
        case 4  % Neuro-Difuso
            az = FuzzyAxis(false, theta1_0, 10.0, 20.0, 5, 80.0, 0.01);
            el = FuzzyAxis(true, theta2_0, 10.0, 20.0, 5, 80.0, 0.01);
    end

    t_vec = (0:n_steps-1)' * dt;
    err1 = zeros(n_steps,1); err2 = zeros(n_steps,1); err_arcsec = zeros(n_steps,1);
    tau1_log = zeros(n_steps,1); tau2_log = zeros(n_steps,1);
    t_last_mpc = -1.0;

    for k = 1:n_steps
        t = t_vec(k);
        [q1r, q1dr, q1ddr, q2r, q2dr, q2ddr] = reference(t, q1_0, q2_0, omega1, omega2);

        [tau_dist, xgddot, tau_wind_true, ~, wind_state, terrain_state] = ...
            disturbance_torque(t, dt, wind_state, terrain_state, params);

        z1 = [state(1) + sigma_enc*randn(); state(2) + sigma_gyro*randn()];
        z2 = [state(3) + sigma_enc*randn(); state(4) + sigma_gyro*randn()];

        ff_seismic1 = params.Fs1 * xgddot;
        ff_seismic2 = params.Fs2 * xgddot;
        ff_wind1 = ekf1.x(3);
        ff_wind2 = ekf2.x(3);

        if controller_type == 3  % MPC: recompute a 100 Hz (dt_mpc=0.01)
            if t - t_last_mpc >= az.dt_mpc - 1e-9
                t_last_mpc = t;
                Ngrid = az.N;
                tgrid = t + (1:Ngrid) * az.dt_mpc;
                q1_ref_seq = q1_0 + omega1 * tgrid;
                qd1_ref_seq = ones(1,Ngrid) * omega1;
                q2_ref_seq = q2_0 + omega2 * tgrid;
                qd2_ref_seq = ones(1,Ngrid) * omega2;
                az.recompute(ekf1.x(1), ekf1.x(2), q1_ref_seq, qd1_ref_seq, ff_seismic1 - ff_wind1);
                el.recompute(ekf2.x(1), ekf2.x(2), q2_ref_seq, qd2_ref_seq, ff_seismic2 - ff_wind2);
            end
            tau1 = az.tau_hold; tau2 = el.tau_hold;
        else
            [tau1, ~] = az.compute(ekf1.x(1), ekf1.x(2), q1r, q1dr, q1ddr, ff_seismic1, ff_wind1, dt);
            [tau2, ~] = el.compute(ekf2.x(1), ekf2.x(2), q2r, q2dr, q2ddr, ff_seismic2, ff_wind2, dt);
        end

        tau = [tau1, tau2];
        qdd = dynamics_forward(state, tau, tau_dist, params);
        state = state + dt * [state(2), qdd(1), state(4), qdd(2)];

        ekf1.predict(tau1, ff_seismic1); ekf1.update(z1);
        ekf2.predict(tau2, ff_seismic2); ekf2.update(z2);

        e1 = state(1) - q1r; e2 = state(3) - q2r;
        err1(k) = e1; err2(k) = e2;
        err_arcsec(k) = hypot(e1, e2) * 206264.806;
        tau1_log(k) = tau1; tau2_log(k) = tau2;
    end

    log.t = t_vec; log.err1 = err1; log.err2 = err2;
    log.err_total_arcsec = err_arcsec;
    log.tau1 = tau1_log; log.tau2 = tau2_log;
    log.label = label;
end

function [q1r, q1dr, q1ddr, q2r, q2dr, q2ddr] = reference(t, q1_0, q2_0, omega1, omega2)
    q1r = q1_0 + omega1*t; q1dr = omega1; q1ddr = 0.0;
    q2r = q2_0 + omega2*t; q2dr = omega2; q2ddr = 0.0;
end

function m = compute_metrics(log, dt)
    t = log.t; err = log.err_total_arcsec;
    phases = {'Basal',[0 5]; 'Coquimbo',[5 25]; 'Chiloe',[25 45]; 'Melipilla',[45 60]};
    m.label = log.label;
    for i = 1:size(phases,1)
        mask = t >= phases{i,2}(1) & t < phases{i,2}(2);
        m.(['rmse_' phases{i,1}]) = sqrt(mean(err(mask).^2));
        m.(['max_' phases{i,1}]) = max(err(mask));
    end
    m.rmse_total = sqrt(mean(err.^2));
    m.max_total = max(err);
    m.effort = sum(log.tau1.^2 + log.tau2.^2) * dt;
    m.chattering = sum(abs(diff(log.tau1))) + sum(abs(diff(log.tau2)));
end
