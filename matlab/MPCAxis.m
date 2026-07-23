classdef MPCAxis < handle
    % MPCAXIS Control Predictivo (MPC) lineal de horizonte deslizante,
    % formulación QP en LOTE resuelta en forma cerrada (mínimos cuadrados
    % regularizados + recorte a límites de par). El modelo de predicción
    % se linealiza en torno a J,b nominales; para el eje de elevación la
    % gravedad se trata como offset constante evaluado en el q actual
    % (válido para el horizonte corto usado, N*dt_mpc <= 150 ms).
    % Las matrices Phi, Gamma y la ganancia batch K_mpc son constantes y
    % se precalculan una sola vez en el constructor.
    properties
        has_gravity, J, b, mgl, N, dt_mpc, tau_hold = 0.0
        Ad, Bd, Phi, GammaOnes, K_mpc
        TAU_MAX = 40.0
    end

    methods
        function obj = MPCAxis(has_gravity, J_nom, b_nom, mgl_nom, N, dt_mpc, Qw, Rw, rho_rate)
            if nargin < 9
                rho_rate = 2.0;
            end
            obj.has_gravity = has_gravity;
            obj.J = J_nom; obj.b = b_nom; obj.mgl = mgl_nom;
            obj.N = N; obj.dt_mpc = dt_mpc;

            n = 2;
            A = [0, 1; 0, -b_nom/J_nom];
            B = [0; 1/J_nom];
            obj.Ad = eye(n) + A * dt_mpc;
            obj.Bd = B * dt_mpc;

            Apows = cell(N+1, 1);
            Apows{1} = eye(n);
            for k = 1:N
                Apows{k+1} = Apows{k} * obj.Ad;
            end

            Phi = zeros(N*n, n);
            Gamma = zeros(N*n, N);
            for k = 0:N-1
                Phi(k*n+1:(k+1)*n, :) = Apows{k+2};
                for j = 0:k
                    Gamma(k*n+1:(k+1)*n, j+1) = Apows{k-j+1} * obj.Bd;
                end
            end

            Qbar = kron(eye(N), diag(Qw));
            D = eye(N) - diag(ones(N-1,1), -1);
            D(1,1) = 0.0;
            Rbar = eye(N) * Rw + rho_rate * Rw * (D' * D);

            H = Gamma' * Qbar * Gamma + Rbar;
            obj.K_mpc = H \ (Gamma' * Qbar);
            obj.Phi = Phi;
            obj.GammaOnes = Gamma * ones(N, 1);
        end

        function tau = recompute(obj, q, qd, q_ref_seq, qd_ref_seq, bias)
            if obj.has_gravity
                grav = obj.mgl * cos(q);
            else
                grav = 0.0;
            end
            c = -(bias + grav);
            x0 = [q; qd];

            N = obj.N;
            Xref = zeros(N*2, 1);
            Xref(1:2:end) = q_ref_seq(:);
            Xref(2:2:end) = qd_ref_seq(:);

            d = obj.Phi * x0 + obj.GammaOnes * c - Xref;
            U = -obj.K_mpc * d;
            U = max(min(U, obj.TAU_MAX), -obj.TAU_MAX);
            obj.tau_hold = U(1);
            tau = obj.tau_hold;
        end
    end
end
