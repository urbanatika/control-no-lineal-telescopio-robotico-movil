function [q1r, q1dr, q1ddr, q2r, q2dr, q2ddr] = reference(t, q1_0, q2_0, omega1, omega2)
% REFERENCE Trayectoria de referencia: tasa sideral en acimut y deriva
% lenta en elevación. Separada de main_simulation.m como función de
% archivo propio para compatibilidad con Octave.
    q1r = q1_0 + omega1*t; q1dr = omega1; q1ddr = 0.0;
    q2r = q2_0 + omega2*t; q2dr = omega2; q2ddr = 0.0;
end
