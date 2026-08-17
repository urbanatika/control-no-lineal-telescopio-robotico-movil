function log = run_one(controller_type, params, dt, n_steps, t_final, ...
                        sigma_enc, sigma_gyro, q1_0, q2_0, omega1, omega2, label)
% RUN_ONE Simula un controlador (1=Adaptativo, 2=SMC, 3=MPC, 4=Neuro-Difuso)
% durante n_steps pasos, con EKF de fusión sensorial por eje. Separada de
% main_simulation.m como función de archivo propio para compatibilidad con
% Octave (que no soporta local functions al final de un script).

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
