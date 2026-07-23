function [tau_dist, xgddot, tau_wind, terr, wind_state, terrain_state] = disturbance_torque(t, dt, wind_state, terrain_state, params)
% DISTURBANCE_TORQUE Combina sismo + viento (Gauss-Markov) + microvibración
% de terreno en el par de perturbación por eje.
%
% wind_state: struct con campos tv1, tv2 (estado del proceso Gauss-Markov,
%             se actualiza in-place vía la salida wind_state de retorno
%             -- en MATLAB, pásese por valor y reasígnese en el caller).
% terrain_state: struct con campos hp_state, hp_prev, lp_state (filtro
%             pasa-banda de la microvibración de terreno).
%
% IMPORTANTE (convención de signos): tau_dist es SUSTRAÍDO ÍNTEGRO de tau
% en dynamics_forward. Para reproducir la física del paper base
% (J*qdd + b*qd + mgl*sin(q) = tau - tau_s + tau_v, ec. 1), el viento entra
% aquí con signo NEGATIVO, de forma que "tau - tau_dist" equivalga a
% "tau - tau_s + tau_v - tau_terreno". La ley de control (ec. 11) usa
% +Fs*xgddot - tau_v_hat, diseñada exactamente para cancelar esto.

    xgddot = seismic_acceleration(t);

    % --- Gauss-Markov del viento (ec. 4) ---
    Q1 = 2 * params.alpha_w1 * params.sigma_w^2;
    Q2 = 2 * params.alpha_w2 * params.sigma_w^2;
    w1 = randn() * sqrt(Q1 / dt);
    w2 = randn() * sqrt(Q2 / dt);
    tv1 = wind_state.tv1 + (-params.alpha_w1 * wind_state.tv1 + w1) * dt;
    tv2 = wind_state.tv2 + (-params.alpha_w2 * wind_state.tv2 + w2) * dt;
    wind_state.tv1 = tv1;
    wind_state.tv2 = tv2;
    tau_wind = [tv1, tv2];

    % --- Microvibración de terreno (ruido pasa-banda 5-50 Hz) ---
    white = 0.35 * randn();
    f_lo = 5.0; f_hi = 50.0;
    a = exp(-2*pi*f_lo*dt);
    hp = white - terrain_state.hp_prev + a * terrain_state.hp_state;
    terrain_state.hp_state = hp;
    terrain_state.hp_prev = white;
    b = exp(-2*pi*f_hi*dt);
    lp = (1 - b) * hp + b * terrain_state.lp_state;
    terrain_state.lp_state = lp;
    terr = lp;

    taud1 = params.Fs1 * xgddot - tv1 + params.terrain_gain1 * terr;
    taud2 = params.Fs2 * xgddot - tv2 + params.terrain_gain2 * terr;
    tau_dist = [taud1, taud2];

    % wind_state y terrain_state se devuelven ya actualizados (MATLAB pasa
    % structs por valor); el caller debe reasignarlos en cada iteración:
    %   [tau_dist, xgddot, tau_wind, terr, wind_state, terrain_state] = ...
    %       disturbance_torque(t, dt, wind_state, terrain_state, params);
end
