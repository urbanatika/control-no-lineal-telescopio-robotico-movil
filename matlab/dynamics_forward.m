function qdd = dynamics_forward(state, tau, tau_dist, params)
% DYNAMICS_FORWARD Aceleración articular real del telescopio móvil (2 GDL).
% state = [q1, q1d, q2, q2d]; tau = [tau1, tau2] (control);
% tau_dist = [taud1, taud2] (sismo + viento + terreno ya combinados,
% ver disturbance_torque.m para la convención de signos).
%
% Ecuaciones de Euler-Lagrange (acoplamiento vía J1(q2)):
%   J1(q2) q1dd + dJ1/dq2 * q1d*q2d + b1 q1d = tau1 - taud1
%   J2 q2dd + 0.5*dJ1/dq2 * q1d^2 + b2 q2d + mgl*cos(q2) = tau2 - taud2

    q1 = state(1); q1d = state(2); q2 = state(3); q2d = state(4);

    J1 = params.J1_0 + params.J1_coupling * cos(q2)^2;
    dJ1 = -2 * params.J1_coupling * cos(q2) * sin(q2);

    coriolis1 = dJ1 * q1d * q2d;
    q1dd = (tau(1) - tau_dist(1) - coriolis1 - params.b1 * q1d) / J1;

    centrifugal2 = 0.5 * dJ1 * q1d^2;
    grav2 = params.mgl * cos(q2);
    q2dd = (tau(2) - tau_dist(2) - centrifugal2 - params.b2 * q2d - grav2) / params.J2;

    qdd = [q1dd, q2dd];
end
