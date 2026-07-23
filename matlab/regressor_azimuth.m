function Y = regressor_azimuth(q1dd_r, q1d_actual)
% REGRESSOR_AZIMUTH Y1 tal que Y1*[J1_0;b1] ~= J1(q2)*q1dd_r + b1*q1d
% (ec. 2 del paper, adaptada al eje de acimut sin gravedad).
    Y = [q1dd_r, q1d_actual];
end
