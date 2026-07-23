| Controlador | RMSE total [arcsec] | Max total [arcsec] | Esfuerzo (∫τ²dt) | Chattering [N·m] | Cómputo/paso [ms] |
|---|---|---|---|---|---|
| Adaptativo-Lyapunov+EKF | 191.7 | 857.0 | 2090.2 | 198692.1 | 0.1620 |
| SMC (Super-Twisting) | 53.9 | 649.0 | 2216.9 | 213611.5 | 0.1148 |
| MPC | 318.7 | 767.8 | 2621.4 | 44911.8 | 0.0515 |
| Neuro-Difuso Adaptativo | 116.0 | 865.1 | 2083.2 | 197749.0 | 0.2777 |

RMSE de estimación de viento del EKF: eje acimut = 2.1356 N·m, eje elevación = 2.0273 N·m.