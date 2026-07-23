function Y = regressor_elevation(q2dd_r, q2d_actual, q2)
% REGRESSOR_ELEVATION Y2 tal que Y2*[J2;b2;mgl] ~= J2*q2dd_r + b2*q2d + mgl*cos(q2)
% (idéntica en estructura a Y(q,qdot,qdot_r,qddot_r)=[qddot_r,qdot,sin(q)]
% del paper, ec. 2, adaptada a cos(q2) por la convención de elevación).
    Y = [q2dd_r, q2d_actual, cos(q2)];
end
