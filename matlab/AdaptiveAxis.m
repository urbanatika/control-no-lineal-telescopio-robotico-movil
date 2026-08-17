classdef AdaptiveAxis < handle
    % ADAPTIVEAXIS Control adaptativo por estabilidad de Lyapunov (ec. 5-12
    % del paper base), por eje. Ley de control (ec. 11) + ley de adaptación
    % paramétrica (ec. 12).
    properties
        has_gravity, theta_hat, Gamma, lam, Kd
        TAU_MAX = 40.0
    end

    methods
        function obj = AdaptiveAxis(has_gravity, theta0, Gamma_diag, lam, Kd)
            obj.has_gravity = has_gravity;
            obj.theta_hat = theta0(:);
            obj.Gamma = diag(Gamma_diag);
            obj.lam = lam;
            obj.Kd = Kd;
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

            tau = Y * obj.theta_hat - obj.Kd * s + ff_seismic - ff_wind;
            tau = max(min(tau, obj.TAU_MAX), -obj.TAU_MAX);

            % Ley de adaptación (ec. 12): thetadot = -Gamma * Y^T * s
            obj.theta_hat = obj.theta_hat - obj.Gamma * Y' * s * dt;
        end
    end
end
