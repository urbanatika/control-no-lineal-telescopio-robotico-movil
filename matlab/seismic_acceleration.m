function xgddot = seismic_acceleration(t)
% SEISMIC_ACCELERATION Señal sintética de aceleración de suelo [m/s^2],
% perfil multi-evento de 60 s (idéntico en estructura al paper base):
%   Fase 1 (0-5s):   ruido basal del desierto
%   Fase 2 (5-25s):  Coquimbo 2015 (Mw8.4) - alta frecuencia, transitorio
%   Fase 3 (25-45s): Chiloé 2016 (Mw7.6) - baja frecuencia, gran amplitud
%   Fase 4 (45-60s): Enjambre Melipilla 2017 (Mw6.9) - ráfagas impulsivas
% Señal SINTÉTICA (no registro real del CSN), ver nota en el informe.

    if t < 5.0
        xgddot = 0.02 * randn();
        return;
    end

    if t < 25.0
        tau = t - 5.0;
        envelope = 3.6 * exp(-((tau - 4.0)^2) / (2 * 3.0^2)) + 0.6;
        hf = sin(2*pi*6.3*tau) + 0.6*sin(2*pi*9.1*tau + 0.4) + 0.4*randn();
        xgddot = envelope * hf;
        return;
    end

    if t < 45.0
        tau = t - 25.0;
        envelope = 4.4 * exp(-((tau - 7.0)^2) / (2 * 6.0^2)) + 0.5;
        lf = sin(2*pi*0.55*tau) + 0.5*sin(2*pi*0.85*tau + 0.9) + 0.25*randn();
        xgddot = envelope * lf;
        return;
    end

    tau = t - 45.0;
    bursts = 0.0;
    for k = 0:5
        t0 = 1.0 + k * 2.3;
        bursts = bursts + 2.8 * exp(-((tau - t0)^2) / (2 * 0.25^2)) * sin(2*pi*7.5*(tau - t0));
    end
    xgddot = bursts + 0.3 * randn();
end
