classdef FuzzyAxis < handle
    % FUZZYAXIS Control Neuro-Difuso Adaptativo: aproximador de base
    % gaussiana (tipo Wang 1993 / ANFIS) que estima la dinámica residual
    % no modelada Delta(e,ed), con ley de adaptación derivada de Lyapunov
    % thetadot = Gamma*xi(x)*s - sigma*theta (sigma-modification).
    properties
        has_gravity, theta_nom, lam, Kd
        centers, width, theta_fuzzy, gamma, sigma_mod
        TAU_MAX = 40.0
    end

    methods
        function obj = FuzzyAxis(has_gravity, theta_nom, lam, Kd, n_mf, gamma, sigma_mod)
            if nargin < 5, n_mf = 5; end
            if nargin < 6, gamma = 80.0; end
            if nargin < 7, sigma_mod = 0.01; end

            obj.has_gravity = has_gravity;
            obj.theta_nom = theta_nom(:);
            obj.lam = lam; obj.Kd = Kd;
            obj.gamma = gamma; obj.sigma_mod = sigma_mod;

            centers_e = linspace(-0.05, 0.05, n_mf);
            centers_ed = linspace(-0.3, 0.3, n_mf);
            [Ce, Ced] = meshgrid(centers_e, centers_ed);
            obj.centers = [Ce(:), Ced(:)];
            obj.width = 0.04;
            obj.theta_fuzzy = zeros(size(obj.centers, 1), 1);
        end

        function xi = basis(obj, e, ed)
            d2 = (obj.centers(:,1) - e).^2 + (obj.centers(:,2) - ed).^2;
            phi = exp(-d2 / (2 * obj.width^2));
            s_sum = sum(phi);
            if s_sum > 1e-9
                xi = phi / s_sum;
            else
                xi = phi;
            end
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

            xi = obj.basis(e, ed);
            delta_hat = obj.theta_fuzzy' * xi;

            tau_base = Y * obj.theta_nom - obj.Kd * s + ff_seismic - ff_wind;
            tau = tau_base - delta_hat;
            tau = max(min(tau, obj.TAU_MAX), -obj.TAU_MAX);

            obj.theta_fuzzy = obj.theta_fuzzy + (obj.gamma * xi * s - obj.sigma_mod * obj.theta_fuzzy) * dt;
        end
    end
end
