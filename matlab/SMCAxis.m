classdef SMCAxis < handle
    % SMCAXIS Control por Modos Deslizantes de 2do orden (Super-Twisting,
    % Moreno & Osorio 2012), robusto a incertidumbre paramétrica y
    % perturbaciones acopladas, con chattering reducido (continuo).
    properties
        has_gravity, theta_nom, lam, k1, k2, w = 0.0
        TAU_MAX = 40.0
    end

    methods
        function obj = SMCAxis(has_gravity, theta_nom, lam, k1, k2)
            obj.has_gravity = has_gravity;
            obj.theta_nom = theta_nom(:);
            obj.lam = lam; obj.k1 = k1; obj.k2 = k2;
        end

        function [tau, s] = compute(obj, q, qd, q_des, qd_des, qdd_des, ...
                                     ff_seismic, ff_wind, dt)
            e = q - q_des;
            ed = qd - qd_des;
            s = ed + obj.lam * e;
            qd_r = qd_des - obj.lam * e;
            qdd_r = qdd_des - obj.lam * ed;

            if obj.has_gravity
                Y = regressor_elevation(qdd_r, qd, q);
            else
                Y = regressor_azimuth(qdd_r, qd);
            end

            tau_eq = Y * obj.theta_nom + ff_seismic - ff_wind;

            u1 = -obj.k1 * sqrt(abs(s)) * sign(s) + obj.w;
            obj.w = obj.w + (-obj.k2 * sign(s)) * dt;

            tau = tau_eq + u1;
            tau = max(min(tau, obj.TAU_MAX), -obj.TAU_MAX);
        end
    end
end
