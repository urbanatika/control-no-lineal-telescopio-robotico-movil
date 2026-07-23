function params = plant_model()
% PLANT_MODEL Parámetros físicos del telescopio móvil (2 GDL: acimut, elevación)
% Réplica exacta de python_sim/model.py (clase PlantParams).
% Escala de banco de laboratorio (NO la escala de la antena ALMA de 100 t
% del paper base -- ver nota de escala en el informe, Sección III).

    params.G = 9.81;

    % Acimut (q1): sin par gravitacional (eje vertical)
    params.J1_0 = 3.2;           % kg m^2
    params.J1_coupling = 0.9;    % kg m^2 (acoplamiento geométrico con q2)
    params.b1 = 0.55;            % N m s/rad

    % Elevación (q2): con par gravitacional
    params.J2 = 1.75;            % kg m^2
    params.b2 = 0.40;            % N m s/rad
    params.m2 = 3.0;             % kg
    params.l2 = 0.15;            % m
    params.mgl = params.m2 * params.G * params.l2;

    % Acoplamiento sísmico -> par articular (análogo vectorial de Fs, ec. 3)
    params.Fs1 = 0.55;           % N m s^2/m
    params.Fs2 = 0.45;           % N m s^2/m

    % Gauss-Markov del viento (ec. 4 del paper)
    params.alpha_w1 = 0.6;       % 1/s
    params.alpha_w2 = 0.6;       % 1/s
    params.sigma_w = 1.8;        % N m

    % Microvibración de terreno/ruedas (nuevo respecto al paper base)
    params.terrain_gain1 = 0.30; % N m / (m/s^2)
    params.terrain_gain2 = 0.25; % N m / (m/s^2)

    % Vector paramétrico real theta = [J1_0, b1, J2, b2, mgl]
    params.theta_true = [params.J1_0, params.b1, params.J2, params.b2, params.mgl];
end
