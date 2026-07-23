# Lista de Materiales (BOM) — Prototipo de Banco de Laboratorio
## Telescopio Móvil: Robot Móvil + Brazo Alt-Azimutal de 2 GDL

Escala: **banco de laboratorio semi-profesional** — apto para validar
experimentalmente las 4 leyes de control (Adaptativo-Lyapunov+EKF, SMC,
MPC, Neuro-Difuso) con datos físicos reales, sin depender del cielo real
para las pruebas de apuntamiento (uso de blanco láser/óptico en interior).

Los parámetros físicos usados en las simulaciones (`python_sim/model.py`,
`matlab/plant_model.m`) fueron dimensionados para ser consistentes con
esta escala (J₁≈3.2 kg·m², J₂≈1.75 kg·m², masa del conjunto brazo+OTA≈3 kg).

---

## 1. Base móvil

| Ítem | Especificación sugerida | Cant. | Costo est. (USD) |
|---|---|---|---|
| Chasis de base móvil | Aluminio/perfil estructural, 4 ruedas, ~40×40 cm | 1 | 120 |
| Motores DC con encoder | 12-24V, reductora, encoder cuadratura 500+ CPR | 4 | 4 × 45 = 180 |
| Driver de motor (puente H) | Doble canal, 20A pico, ej. Cytron/VNH5019 | 2 | 2 × 25 = 50 |
| Ruedas | Omnidireccionales (mecanum) o diferenciales + rueda loca | 4 | 4 × 20 = 80 |
| Batería LiPo/Li-ion + BMS | 12-24V, ≥5000 mAh, con protección de carga/descarga | 1 | 90 |
| Regulador de voltaje (buck) | 24V→12V/5V, ≥5A | 2 | 2 × 15 = 30 |
| **Subtotal base móvil** | | | **≈ 550** |

## 2. Brazo robótico alt-azimutal (2-3 GDL)

| Ítem | Especificación sugerida | Cant. | Costo est. (USD) |
|---|---|---|---|
| Actuador acimut (eje vertical) | Servo de torque controlado o motor DC + reductora armónica, encoder absoluto (magnético, 14-bit) | 1 | 180 |
| Actuador elevación | Igual especificación, con freno mecánico opcional (mantiene posición sin consumo) | 1 | 180 |
| Estructura del brazo | Perfil de aluminio 20×20 mm o piezas impresas en 3D (PETG/nylon) | 1 set | 60 |
| Contrapeso de elevación | Masa ajustable para balancear el CG (reduce par gravitacional neto) | 1 | 20 |
| Sensor de par (opcional, validación) | Celda de carga/torque en el eje de elevación | 1 | 70 |
| Rodamientos de precisión | Radiales + axiales para ambos ejes | 4 | 4 × 12 = 48 |
| **Subtotal brazo** | | | **≈ 558** |

## 3. Carga útil óptica (instrumento simulado)

| Ítem | Especificación sugerida | Cant. | Costo est. (USD) |
|---|---|---|---|
| Puntero láser de baja potencia (clase 1-2) | Para materializar la línea de visión (boresight) en pruebas de interior | 1 | 15 |
| Fotodiodo/PSD (Position Sensitive Detector) o cámara de guiado | Mide el error de apuntamiento respecto a un blanco fijo — reemplaza al "cielo real" en el banco de pruebas | 1 | 90 (PSD) / 60 (cámara USB + OpenCV) |
| Tubo óptico ligero (mock OTA) | Simula masa/inercia/CG de un telescopio pequeño (p. ej. tubo de PVC con lastre) | 1 | 25 |
| **Subtotal carga útil** | | | **≈ 130** |

## 4. Sensórica

| Ítem | Especificación sugerida | Cant. | Costo est. (USD) |
|---|---|---|---|
| IMU 9 ejes (acelerómetro+giro+magnetómetro) | Ej. BNO055 o ICM-20948, en la base móvil (mide vibración/inclinación real) | 1 | 35 |
| Acelerómetro/sismómetro de base | Acelerómetro MEMS de baja frecuencia (≤50 Hz), fijado a la fundación del banco de pruebas — análogo al sismómetro piezoeléctrico del paper base | 1 | 40 |
| Encoders articulares | Ya incluidos en actuadores de acimut/elevación (ver sección 2) | — | — |
| Giroscopio de fibra óptica (FOG) — *opcional, alta fidelidad* | Sustituto de alta precisión del giroscopio MEMS para validar el EKF con ruido más realista | 1 | 300+ (opcional) |
| **Subtotal sensórica** | | | **≈ 75 (+300 opcional)** |

## 5. Cómputo y electrónica de control

| Ítem | Especificación sugerida | Cant. | Costo est. (USD) |
|---|---|---|---|
| Microcontrolador de tiempo real | STM32F4/F7 (lazo de control a 1 kHz, lectura de encoders, PWM a drivers) | 1 | 25 |
| SBC (Single Board Computer) | Raspberry Pi 4/5 u orange/Jetson Nano — corre EKF, MPC (QP), lógica difusa/ANFIS de mayor carga computacional | 1 | 90 |
| Módulo de comunicación SBC↔MCU | UART/SPI o CAN bus | 1 | 15 |
| Cableado, conectores, protoboard/PCB | — | 1 set | 40 |
| **Subtotal cómputo** | | | **≈ 170** |

## 6. Banco de pruebas (generación de perturbaciones controladas)

| Ítem | Especificación sugerida | Cant. | Costo est. (USD) |
|---|---|---|---|
| Mesa vibratoria pequeña (shaker) | Actuador de vibración de bajo costo (ej. basado en altavoz de exceso/woofer + amplificador, o shaker comercial de escritorio) para emular el perfil sísmico multi-evento (Coquimbo/Chiloé/Melipilla) de forma reproducible | 1 | 150 |
| Ventiladores de flujo variable | 2-3 ventiladores con control PWM de velocidad, para emular ráfagas de viento estocásticas (Gauss-Markov) sobre la carga útil | 3 | 3 × 20 = 60 |
| Plataforma de aislamiento/fundación | Base rígida sobre la que se monta el shaker y el robot, para transmitir la vibración de forma controlada | 1 | 50 |
| **Subtotal banco de pruebas** | | | **≈ 260** |

---

## Resumen de costos

| Subsistema | Costo estimado (USD) |
|---|---|
| Base móvil | 550 |
| Brazo robótico | 558 |
| Carga útil óptica | 130 |
| Sensórica | 75 (+300 opcional FOG) |
| Cómputo y electrónica | 170 |
| Banco de pruebas | 260 |
| **TOTAL (configuración base)** | **≈ 1 743 USD** |
| **TOTAL (con FOG de alta fidelidad)** | **≈ 2 043 USD** |

*Nota: precios referenciales de mercado (componentes genéricos/hobby-grade
a semi-profesional), sujetos a variación por proveedor y disponibilidad en
Chile (importación). No incluye herramientas de taller, impresora 3D
(si no se dispone de una), ni tiempo de mano de obra de ensamblaje.*

---

## Hoja de ruta de implementación física sugerida

1. **Fase 1 — Validación de subsistemas aislados**: ensamblar y caracterizar
   por separado la base móvil (control diferencial/mecanum básico) y el
   brazo de 2 GDL (identificación de J, b reales por eje mediante ensayos
   de par-respuesta, para comparar contra los valores nominales asumidos
   en la simulación).
2. **Fase 2 — Integración de sensórica y EKF**: montar IMU, acelerómetro de
   base y encoders; verificar en banco estático que el EKF converge y
   estima correctamente posición/velocidad limpias antes de cerrar el lazo
   de control.
3. **Fase 3 — Cierre de lazo con controlador más simple (SMC o Adaptativo)**:
   implementar primero en el SBC/MCU la ley más robusta a errores de
   modelado (según los resultados de simulación, SMC), validar seguimiento
   de una referencia lenta (análoga a la tasa sideral) sin perturbación.
4. **Fase 4 — Pruebas con perturbación controlada**: activar el shaker con
   el perfil multi-evento sintético y los ventiladores, replicar las
   métricas de la Sección de Resultados del informe (RMSE, error máximo
   por fase) con datos físicos reales.
5. **Fase 5 — Comparación de las 4 leyes en banco físico**: repetir el
   protocolo de la Fase 4 para MPC y Neuro-Difuso, validando si el orden de
   desempeño observado en simulación (SMC > Neuro-Difuso > Adaptativo > MPC,
   ver informe) se sostiene con dinámica e incertidumbre reales del
   prototipo.
