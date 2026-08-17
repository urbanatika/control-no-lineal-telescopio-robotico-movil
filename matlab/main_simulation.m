% MAIN_SIMULATION Simulación comparativa de las 4 leyes de control sobre
% el telescopio móvil de 2 GDL (acimut, elevación), bajo perturbación
% multi-evento idéntica (mismo seed) para cada ley -> comparación pareada.
%
% Réplica exacta (misma estructura, parámetros y ganancias) del script
% python_sim/main_simulation.py. Verificado en GNU Octave 11.3.0 (funciones
% locales run_one/reference/compute_metrics separadas en sus propios
% archivos .m para compatibilidad MATLAB/Octave -- Octave no soporta
% "local functions" al final de un script como MATLAB >= R2016b).
%
% Salidas: struct `logs` (uno por controlador) y `metrics` (tabla resumen),
% además de todas las figuras comparativas en ../figures/.

clear; clc; close all;
% (el seed se fija dentro de run_one, una vez por controlador -- ver abajo)

DT = 1e-3;
T_FINAL = 60.0;
N_STEPS = round(T_FINAL / DT);

SIGMA_ENC = 0.002;   % rad
SIGMA_GYRO = 0.01;   % rad/s

Q1_0 = deg2rad(45.0);
Q2_0 = deg2rad(30.0);
OMEGA1 = 7.292e-5;   % rad/s (tasa sideral, acimut)
OMEGA2 = 2.0e-5;     % rad/s (deriva lenta, elevación)

params = plant_model();

controller_labels = {'Adaptativo-Lyapunov+EKF', 'SMC (Super-Twisting)', 'MPC', 'Neuro-Difuso Adaptativo'};
logs = struct();
metrics = struct([]);

for c = 1:4
    label = controller_labels{c};
    fprintf('-> Corriendo %s ...\n', label);
    log = run_one(c, params, DT, N_STEPS, T_FINAL, SIGMA_ENC, SIGMA_GYRO, ...
                  Q1_0, Q2_0, OMEGA1, OMEGA2, label);
    field = genvarname(label);   %#ok<DEPGENAM> -- compatible con MATLAB y Octave
    logs.(field) = log;
    metrics(c) = compute_metrics(log, DT);
end

save('sim_results_matlab.mat', 'logs', 'metrics');

if exist('struct2table', 'file') || exist('struct2table', 'builtin')
    disp(struct2table(metrics));
else
    for c = 1:numel(metrics)
        m = metrics(c);
        fprintf('%-28s rmse_total=%.4f max_total=%.4f effort=%.4f chattering=%.4f\n', ...
                m.label, m.rmse_total, m.max_total, m.effort, m.chattering);
    end
end
