classdef AxisEKF < handle
    % AXISEKF Filtro de Kalman Extendido por eje, extensión directa de las
    % ecuaciones 13-19 del paper base. Estado x = [q; qdot; tau_v].
    properties
        dt, J, b, mgl, alpha, has_gravity
        x       % [q; qdot; tau_v]
        P
        Q, R, H
    end

    methods
        function obj = AxisEKF(dt, J_nom, b_nom, mgl_nom, alpha_w, has_gravity, ...
                                sigma_encoder, sigma_gyro, q_process)
            if nargin < 9
                q_process = [1e-8, 1e-5, 4.0];
            end
            obj.dt = dt; obj.J = J_nom; obj.b = b_nom; obj.mgl = mgl_nom;
            obj.alpha = alpha_w; obj.has_gravity = has_gravity;

            obj.x = zeros(3,1);
            obj.P = diag([1e-3, 1e-2, 25.0]);
            obj.Q = diag(q_process);
            obj.R = diag([sigma_encoder^2, sigma_gyro^2]);
            obj.H = [1 0 0; 0 1 0];
        end

        function xdot = f(obj, x, tau, tau_seismic)
            q = x(1); qd = x(2); tv = x(3);
            if obj.has_gravity
                grav = obj.mgl * cos(q);
            else
                grav = 0.0;
            end
            qdd = (tau - tau_seismic - obj.b*qd - grav + tv) / obj.J;
            tvd = -obj.alpha * tv;
            xdot = [qd; qdd; tvd];
        end

        function F = jacobian(obj, x)
            q = x(1);
            F = zeros(3,3);
            F(1,2) = 1.0;
            if obj.has_gravity
                F(2,1) = obj.mgl * sin(q) / obj.J;
            end
            F(2,2) = -obj.b / obj.J;
            F(2,3) = 1.0 / obj.J;
            F(3,3) = -obj.alpha;
        end

        function predict(obj, tau, tau_seismic)
            xdot = obj.f(obj.x, tau, tau_seismic);
            obj.x = obj.x + xdot * obj.dt;
            Phi = eye(3) + obj.jacobian(obj.x) * obj.dt;
            obj.P = Phi * obj.P * Phi' + obj.Q;
        end

        function update(obj, z_meas)
            y = z_meas - obj.H * obj.x;
            S = obj.H * obj.P * obj.H' + obj.R;
            K = obj.P * obj.H' / S;
            obj.x = obj.x + K * y;
            obj.P = (eye(3) - K*obj.H) * obj.P;
        end
    end
end
