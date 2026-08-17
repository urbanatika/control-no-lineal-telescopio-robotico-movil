function m = compute_metrics(log, dt)
% COMPUTE_METRICS Métricas comparativas por fase (Basal/Coquimbo/Chiloé/
% Melipilla) y totales: RMSE, máximo, esfuerzo de control y chattering.
% Separada de main_simulation.m como función de archivo propio para
% compatibilidad con Octave.
    t = log.t; err = log.err_total_arcsec;
    phases = {'Basal',[0 5]; 'Coquimbo',[5 25]; 'Chiloe',[25 45]; 'Melipilla',[45 60]};
    m.label = log.label;
    for i = 1:size(phases,1)
        mask = t >= phases{i,2}(1) & t < phases{i,2}(2);
        m.(['rmse_' phases{i,1}]) = sqrt(mean(err(mask).^2));
        m.(['max_' phases{i,1}]) = max(err(mask));
    end
    m.rmse_total = sqrt(mean(err.^2));
    m.max_total = max(err);
    m.effort = sum(log.tau1.^2 + log.tau2.^2) * dt;
    m.chattering = sum(abs(diff(log.tau1))) + sum(abs(diff(log.tau2)));
end
