"""
Genera los diagramas de diseño del prototipo físico (banco de laboratorio):
  fig9  - Diagrama de bloques del sistema (sensórica/cómputo/actuación)
  fig10 - Esquema eléctrico y de buses de comunicación
  fig11 - Layout mecánico del robot (vista lateral)
  fig12 - Disposición del banco de pruebas (perturbaciones controladas)

Complementan la hoja de ruta de materiales/BOM.md y la Sección de
Materiales del informe. Salidas -> ../figures/ como PNG (300dpi) y PDF
vectorial, mismo estilo que make_figures.py.
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle, Rectangle, Wedge
from matplotlib.lines import Line2D

FIG_DIR = "../figures"

NAVY = "#1B2A4A"
ACCENT = "#D62728"
GREEN = "#2CA02C"
GRAY = "#555555"
LIGHT = "#EEF1F8"
LIGHT_RED = "#FCEBEA"
LIGHT_GREEN = "#EAF7EA"


def save(fig, name):
    fig.savefig(f"{FIG_DIR}/{name}.png", dpi=300, bbox_inches="tight")
    fig.savefig(f"{FIG_DIR}/{name}.pdf", bbox_inches="tight")
    plt.close(fig)
    print(f"  guardado {name}.png / .pdf")


def box(ax, x, y, w, h, text, fc=LIGHT, ec=NAVY, fontsize=8.5, textcolor="black", lw=1.3, zorder=3):
    b = FancyBboxPatch((x, y), w, h,
                        boxstyle="round,pad=0.02,rounding_size=0.08",
                        linewidth=lw, edgecolor=ec, facecolor=fc, zorder=zorder)
    ax.add_patch(b)
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center",
            fontsize=fontsize, color=textcolor, zorder=zorder + 1, linespacing=1.35)
    return (x, y, w, h)


def group_box(ax, x, y, w, h, label, ec=GRAY):
    b = FancyBboxPatch((x, y), w, h,
                        boxstyle="round,pad=0.03,rounding_size=0.1",
                        linewidth=1.1, linestyle="dashed", edgecolor=ec,
                        facecolor="none", zorder=1)
    ax.add_patch(b)
    ax.text(x + 0.12, y + h - 0.05, label, ha="left", va="top",
            fontsize=8, color=ec, style="italic", zorder=2,
            bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none"))


def arrow(ax, p1, p2, text=None, color=GRAY, lw=1.4, ls="-", rad=0.0, fontsize=7.5,
          label_at=None):
    a = FancyArrowPatch(p1, p2, arrowstyle="-|>", mutation_scale=12,
                         linewidth=lw, color=color, linestyle=ls,
                         connectionstyle=f"arc3,rad={rad}", zorder=2)
    ax.add_patch(a)
    if text:
        if label_at is not None:
            mx, my = label_at
        else:
            mx, my = (p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2 + 0.12
        ax.text(mx, my, text, ha="center", va="bottom", fontsize=fontsize, color=color,
                 zorder=5, bbox=dict(boxstyle="round,pad=0.12", fc="white", ec="none", alpha=0.92))


# ---------------------------------------------------------------------------
# Fig 9 - Diagrama de bloques del sistema
# ---------------------------------------------------------------------------
def fig_block_diagram():
    fig, ax = plt.subplots(figsize=(9.5, 8))
    ax.set_xlim(0, 10.5)
    ax.set_ylim(0, 12.2)
    ax.axis("off")
    ax.set_title("Diagrama de bloques del sistema — prototipo de banco de laboratorio",
                  fontsize=11, fontweight="bold", color=NAVY, pad=10)

    # Perturbaciones (arriba, grupo)
    group_box(ax, 0.3, 10.3, 9.9, 1.6, "Perturbaciones controladas (banco de pruebas, Fig. 12)")
    box(ax, 0.6, 10.55, 3.0, 1.1, "Mesa vibratoria (shaker)\nperfil sísmico multi-evento", fc=LIGHT_RED, ec=ACCENT)
    box(ax, 3.9, 10.55, 3.0, 1.1, "Ventiladores PWM\nráfagas de viento (Gauss-Markov)", fc=LIGHT_RED, ec=ACCENT)
    box(ax, 7.2, 10.55, 2.7, 1.1, "Microvibración\nrueda-terreno", fc=LIGHT_RED, ec=ACCENT)

    # Sensórica
    group_box(ax, 0.3, 8.1, 9.9, 1.85, "Sensórica")
    box(ax, 0.6, 8.35, 2.9, 1.3, "Acelerómetro de base\n(sismómetro, ≤50 Hz)", fc=LIGHT_GREEN, ec=GREEN)
    box(ax, 3.75, 8.35, 2.9, 1.3, "IMU 9 ejes\n(base móvil)", fc=LIGHT_GREEN, ec=GREEN)
    box(ax, 6.9, 8.35, 2.9, 1.3, "Encoders articulares\n(acimut, elevación)", fc=LIGHT_GREEN, ec=GREEN)

    # Computo: MCU y SBC
    group_box(ax, 0.3, 5.3, 9.9, 2.45, "Cómputo")
    box(ax, 0.6, 5.7, 4.3, 1.75,
        "MCU tiempo real (STM32F4/F7)\nlazo determinístico a 1 kHz:\nlectura encoders/IMU + PWM",
        fc=LIGHT, ec=NAVY, fontsize=8.5)
    box(ax, 5.5, 5.7, 4.3, 1.75,
        "SBC (Raspberry Pi / Jetson)\nEKF de fusión sensorial\nLey de control seleccionable:\nAdaptativo-Lyapunov / SMC /\nMPC / Neuro-Difuso",
        fc=LIGHT, ec=NAVY, fontsize=8.2)
    arrow(ax, (4.9, 6.575), (5.5, 6.575), "UART / SPI / CAN", rad=0.0)
    arrow(ax, (5.5, 6.3), (4.9, 6.3), "", rad=0.0)

    # Actuación
    group_box(ax, 0.3, 2.9, 9.9, 1.85, "Actuación")
    box(ax, 0.6, 3.15, 2.9, 1.3, "Driver puente H\n(motor acimut)", fc="#FFF4DE", ec="#B8860B")
    box(ax, 3.75, 3.15, 2.9, 1.3, "Driver puente H\n(motor elevación)", fc="#FFF4DE", ec="#B8860B")
    box(ax, 6.9, 3.15, 2.9, 1.3, "Drivers motores\ntracción base (×4)", fc="#FFF4DE", ec="#B8860B")

    # Planta física
    group_box(ax, 0.3, 0.15, 9.9, 2.3, "Planta física (Fig. 11)")
    box(ax, 0.6, 0.4, 3.1, 1.8, "Base móvil\n(chasis + ruedas)", fc="white", ec=GRAY)
    box(ax, 4.0, 0.4, 2.9, 1.8, "Brazo alt-azimutal\n2 GDL", fc="white", ec=GRAY)
    box(ax, 7.15, 0.4, 2.65, 1.8, "Carga útil óptica\n(láser + PSD/cámara)", fc="white", ec=GRAY)

    # Flechas verticales principales
    arrow(ax, (2.05, 10.55), (2.05, 9.65), "")
    arrow(ax, (5.25, 10.55), (5.25, 9.65), "")
    arrow(ax, (8.55, 10.55), (8.55, 9.65), "")
    arrow(ax, (2.05, 8.35), (2.75, 7.45), "τ_sismo\n(feedforward)")
    arrow(ax, (5.2, 8.35), (5.2, 7.45), "IMU (opcional)")
    arrow(ax, (8.35, 8.35), (7.65, 7.45), "q_enc, q̇_gyro")
    arrow(ax, (2.75, 5.7), (2.05, 4.45), "PWM + DIR")
    arrow(ax, (5.2, 5.7), (5.2, 4.45), "PWM + DIR")
    arrow(ax, (7.65, 5.7), (8.35, 4.45), "PWM + DIR (×4)")
    arrow(ax, (2.05, 3.15), (2.05, 2.2), "par acimut")
    arrow(ax, (5.2, 3.15), (5.2, 2.2), "par elevación")
    arrow(ax, (8.35, 3.15), (8.35, 2.2), "tracción")
    arrow(ax, (5.85, 1.3), (7.15, 1.3), "")

    # Realimentación óptica (validación, opcional)
    arrow(ax, (8.475, 2.2), (9.9, 6.0), "error de apuntamiento\n(realimentación de validación)",
          color=ACCENT, ls="dashed", rad=-0.25)

    fig.tight_layout()
    save(fig, "fig9_diagrama_bloques")


# ---------------------------------------------------------------------------
# Fig 10 - Esquema eléctrico y de buses
# ---------------------------------------------------------------------------
def fig_electrical_schematic():
    fig, ax = plt.subplots(figsize=(10, 8.2))
    ax.set_xlim(0, 11)
    ax.set_ylim(1.6, 11)
    ax.axis("off")
    ax.set_title("Esquema eléctrico y de buses de comunicación",
                  fontsize=11, fontweight="bold", color=NAVY, pad=10)

    # --- Alimentación ---
    group_box(ax, 0.3, 9.15, 10.4, 1.55, "Alimentación")
    box(ax, 0.6, 9.4, 2.4, 1.0, "Batería\nLiPo/Li-ion + BMS\n12-24 V", fc=LIGHT_RED, ec=ACCENT)
    box(ax, 3.6, 9.4, 2.6, 1.0, "Buck 24V→12V/5V\n(regulador ×2)", fc=LIGHT_RED, ec=ACCENT)
    arrow(ax, (3.0, 9.9), (3.6, 9.9), "")

    rail_y = 9.0
    ax.plot([0.6, 10.4], [rail_y, rail_y], color=GRAY, lw=2.2, zorder=1)
    ax.text(10.45, rail_y, "riel 12/5 V", fontsize=7.5, color=GRAY, va="center")
    arrow(ax, (4.9, 9.4), (4.9, rail_y), "")

    # --- Cómputo y drivers (una sola fila, con huecos reales entre cajas) ---
    row2_y, row2_h = 6.7, 1.8
    drv_arm = box(ax, 0.6, row2_y, 2.0, row2_h, "Drivers puente H\n(motores brazo)", fc="#FFF4DE", ec="#B8860B")
    mcu = box(ax, 3.1, row2_y, 2.0, row2_h, "MCU\n(STM32,\nlazo 1 kHz)", fc=LIGHT, ec=NAVY)
    sbc = box(ax, 5.6, row2_y, 2.0, row2_h, "SBC\n(RPi/Jetson)\nEKF+control", fc=LIGHT, ec=NAVY)
    drv_base = box(ax, 8.1, row2_y, 2.3, row2_h, "Drivers motores\nbase (×4)", fc="#FFF4DE", ec="#B8860B")

    for cx in [1.6, 4.1, 6.6, 9.25]:
        arrow(ax, (cx, rail_y), (cx, row2_y + row2_h), "")

    mid_y = row2_y + row2_h / 2
    # hueco 1 (2.6-3.1): MCU -> drivers brazo
    arrow(ax, (3.1, mid_y), (2.6, mid_y), "PWM\n+DIR", label_at=(2.85, mid_y + 0.35), fontsize=7)
    # hueco 2 (5.1-5.6): MCU <-> SBC
    arrow(ax, (5.1, mid_y + 0.3), (5.6, mid_y + 0.3), "", rad=0.0)
    arrow(ax, (5.6, mid_y - 0.3), (5.1, mid_y - 0.3), "", rad=0.0)
    ax.text(5.35, mid_y + 0.75, "UART/\nSPI/CAN", ha="center", va="bottom", fontsize=7, color=GRAY,
            bbox=dict(boxstyle="round,pad=0.12", fc="white", ec="none", alpha=0.92), zorder=5)
    # MCU -> drivers base: arco POR ENCIMA de SBC (evita cruzar su caja),
    # aplanado para no salirse del hueco libre entre la fila 2 y el riel.
    arrow(ax, (4.3, row2_y + row2_h), (9.0, row2_y + row2_h), "", rad=-0.09, lw=1.3)
    ax.text(6.7, row2_y + row2_h + 0.22, "PWM + DIR (base, ×4)", ha="center", fontsize=7, color=GRAY,
            bbox=dict(boxstyle="round,pad=0.12", fc="white", ec="none", alpha=0.92), zorder=5)

    # --- Sensórica (bajo MCU y SBC) + motores (bajo los drivers) ---
    row3_y, row3_h = 4.3, 1.4
    box(ax, 0.6, row3_y, 2.0, row3_h, "Motores brazo\n(acimut/elevación)", fc="white", ec=GRAY)
    box(ax, 3.1, row3_y, 0.9, row3_h, "Encoders\nacimut/\nelevación", fc=LIGHT_GREEN, ec=GREEN, fontsize=7)
    box(ax, 4.15, row3_y, 0.95, row3_h, "Acelerómetro\nbase\n(ADC)", fc=LIGHT_GREEN, ec=GREEN, fontsize=7)
    box(ax, 5.6, row3_y, 0.95, row3_h, "IMU 9 ejes\n(I2C/SPI)", fc=LIGHT_GREEN, ec=GREEN, fontsize=7)
    box(ax, 6.65, row3_y, 0.95, row3_h, "Cámara/PSD\n(USB)", fc=LIGHT_GREEN, ec=GREEN, fontsize=7)
    box(ax, 8.1, row3_y, 2.3, row3_h, "Motores base ×4\n(tracción)", fc="white", ec=GRAY)

    arrow(ax, (1.6, row2_y), (1.6, row3_y + row3_h), "par")
    arrow(ax, (3.55, row3_y + row3_h), (3.7, row2_y), "cuadratura A/B", fontsize=6.8)
    arrow(ax, (4.6, row3_y + row3_h), (4.4, row2_y), "señal + ADC", fontsize=6.8)
    arrow(ax, (6.05, row3_y + row3_h), (5.9, row2_y), "I2C/SPI", fontsize=6.8)
    arrow(ax, (7.1, row3_y + row3_h), (7.0, row2_y), "USB", fontsize=6.8)
    arrow(ax, (9.25, row2_y), (9.25, row3_y + row3_h), "tracción")

    # --- Tierra común ---
    gnd_y = 2.0
    ax.plot([0.6, 10.4], [gnd_y, gnd_y], color="black", lw=1.6, zorder=1)
    ax.text(10.45, gnd_y, "GND común", fontsize=7.5, va="center")
    for cx in [1.6, 3.55, 4.6, 6.05, 7.1, 9.25]:
        ax.plot([cx, cx], [gnd_y, row3_y], color="black", lw=0.8, ls=":", zorder=1)

    fig.tight_layout()
    save(fig, "fig10_esquema_electrico")


# ---------------------------------------------------------------------------
# Fig 11 - Layout mecánico (vista lateral)
# ---------------------------------------------------------------------------
def fig_mechanical_layout():
    fig, ax = plt.subplots(figsize=(9, 6.5))
    ax.set_xlim(-1, 11)
    ax.set_ylim(-1, 8.5)
    ax.axis("off")
    ax.set_aspect("equal")
    ax.set_title("Layout mecánico del robot — vista lateral (no a escala)",
                  fontsize=11, fontweight="bold", color=NAVY, pad=10)

    # Suelo
    ax.plot([-1, 11], [0, 0], color="black", lw=1.2)
    for x in range(-1, 11):
        ax.plot([x, x - 0.3], [0, -0.3], color="black", lw=0.6)

    # Base móvil (chasis)
    chassis = Rectangle((1.0, 0.55), 4.0, 1.0, fc=LIGHT, ec=NAVY, lw=1.4, zorder=3)
    ax.add_patch(chassis)
    ax.text(3.0, 1.05, "Chasis base móvil", ha="center", va="center", fontsize=8.5)
    for wx in [1.8, 4.2]:
        w = Circle((wx, 0.55), 0.42, fc="#333333", ec="black", zorder=4)
        ax.add_patch(w)
    ax.text(3.0, -0.55, "Ruedas (mecanum / diferencial)", ha="center", fontsize=7.5, color=GRAY)

    # Acelerómetro de base + IMU
    box(ax, 1.15, 1.65, 1.5, 0.55, "Acelerómetro\nbase", fc=LIGHT_GREEN, ec=GREEN, fontsize=6.8)
    box(ax, 3.35, 1.65, 1.5, 0.55, "IMU 9 ejes", fc=LIGHT_GREEN, ec=GREEN, fontsize=6.8)

    # Actuador acimut (eje vertical) sobre la base
    az_x, az_y = 3.0, 2.2
    az = Rectangle((az_x - 0.4, az_y), 0.8, 0.9, fc="#FFF4DE", ec="#B8860B", lw=1.3, zorder=3)
    ax.add_patch(az)
    ax.text(az_x, az_y + 0.45, "Actuador\nacimut", ha="center", va="center", fontsize=6.8)
    ax.annotate("", xy=(az_x + 0.75, az_y + 1.15), xytext=(az_x + 0.4, az_y + 0.8),
                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=0.6", color=GRAY, lw=1.0))
    ax.text(az_x + 1.0, az_y + 1.0, "q₁ (acimut)", fontsize=7, color=GRAY)

    # Brazo hasta el eje de elevación
    arm_base = (az_x, az_y + 0.9)
    el_x, el_y = az_x, az_y + 2.0
    ax.plot([arm_base[0], el_x], [arm_base[1], el_y], color=GRAY, lw=3, zorder=2)
    elv = Circle((el_x, el_y), 0.35, fc="#FFF4DE", ec="#B8860B", lw=1.3, zorder=3)
    ax.add_patch(elv)
    ax.text(el_x, el_y - 0.75, "Actuador\nelevación", ha="center", fontsize=6.8)
    ax.annotate("", xy=(el_x + 0.55, el_y + 0.35), xytext=(el_x + 0.1, el_y + 0.45),
                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=0.6", color=GRAY, lw=1.0))
    ax.text(el_x + 0.7, el_y + 0.55, "q₂ (elevación)", fontsize=7, color=GRAY)

    # Tubo óptico (OTA simulado) + contrapeso, inclinado ~20°
    import numpy as np
    ang = np.deg2rad(20)
    L = 2.6
    tip = (el_x + L * np.cos(ang), el_y + L * np.sin(ang))
    ax.plot([el_x, tip[0]], [el_y, tip[1]], color=NAVY, lw=6, solid_capstyle="round", zorder=2)
    ax.text((el_x + tip[0]) / 2 + 0.3, (el_y + tip[1]) / 2 + 0.35, "Tubo óptico\n(mock OTA)",
            fontsize=7, color=NAVY, ha="center")

    cw = (el_x - 1.1 * np.cos(ang), el_y - 1.1 * np.sin(ang))
    ax.add_patch(Circle(cw, 0.28, fc="#888888", ec="black", zorder=3))
    ax.text(cw[0] - 0.2, cw[1] - 0.55, "Contrapeso", fontsize=7, color=GRAY, ha="center")

    # Carga útil óptica en la punta
    box(ax, tip[0] - 0.05, tip[1] - 0.05, 1.5, 0.7, "Láser +\nPSD/cámara", fc=LIGHT_RED, ec=ACCENT, fontsize=6.8)
    ax.annotate("", xy=(tip[0] + 2.3, tip[1] + 0.9), xytext=(tip[0] + 0.7, tip[1] + 0.3),
                arrowprops=dict(arrowstyle="->", color=ACCENT, lw=1.2, ls="dashed"))
    ax.text(tip[0] + 2.4, tip[1] + 0.95, "línea de visión\n→ blanco fijo (Fig. 12)",
            fontsize=7, color=ACCENT)

    # Anotaciones de parámetros físicos
    ax.text(9.6, 7.6,
            "J₁ ≈ 3.2 kg·m²\nJ₂ ≈ 1.75 kg·m²\nm₂ ≈ 3 kg, l₂ ≈ 0.15 m",
            fontsize=7.5, color=GRAY, ha="left", va="top",
            bbox=dict(boxstyle="round,pad=0.3", fc="white", ec=GRAY, lw=0.8))

    fig.tight_layout()
    save(fig, "fig11_layout_mecanico")


# ---------------------------------------------------------------------------
# Fig 12 - Banco de pruebas (perturbaciones controladas)
# ---------------------------------------------------------------------------
def fig_test_bench():
    fig, ax = plt.subplots(figsize=(9.5, 6))
    ax.set_xlim(-1, 12)
    ax.set_ylim(-1, 6.5)
    ax.axis("off")
    ax.set_aspect("equal")
    ax.set_title("Disposición del banco de pruebas — perturbaciones controladas",
                  fontsize=11, fontweight="bold", color=NAVY, pad=10)

    # Suelo
    ax.plot([-1, 12], [0, 0], color="black", lw=1.2)

    # Plataforma de aislamiento/fundación
    found = Rectangle((0.5, 0.3), 4.5, 0.5, fc="#DDDDDD", ec="black", lw=1.2, zorder=2)
    ax.add_patch(found)
    ax.text(2.75, 0.55, "Plataforma de aislamiento / fundación", ha="center", va="center", fontsize=7.5)

    # Shaker (mesa vibratoria)
    shaker = Rectangle((1.2, 0.8), 3.1, 0.35, fc="#FFF4DE", ec="#B8860B", lw=1.3, zorder=3)
    ax.add_patch(shaker)
    ax.text(2.75, 0.975, "Mesa vibratoria (shaker)", ha="center", va="center", fontsize=7)
    for x in [1.6, 3.9]:
        ax.annotate("", xy=(x, 1.13), xytext=(x, 0.83),
                    arrowprops=dict(arrowstyle="->", color=ACCENT, lw=1.3))

    # Robot móvil + brazo (bloque simplificado sobre el shaker)
    robot = Rectangle((1.6, 1.15), 2.3, 0.7, fc=LIGHT, ec=NAVY, lw=1.4, zorder=4)
    ax.add_patch(robot)
    ax.text(2.75, 1.5, "Robot móvil\n+ brazo 2 GDL", ha="center", va="center", fontsize=7)
    ax.annotate("perfil sísmico multi-evento\n(Coquimbo/Chiloé/Melipilla)",
                xy=(2.75, 1.85), xytext=(2.75, 2.35),
                ha="center", fontsize=6.8, color=ACCENT,
                arrowprops=dict(arrowstyle="->", color=ACCENT, lw=1.0))
    ax.plot([3.9, 5.6], [1.85, 3.3], color=NAVY, lw=5, solid_capstyle="round", zorder=4)
    payload = Rectangle((5.5, 3.15), 1.0, 0.5, fc=LIGHT_RED, ec=ACCENT, lw=1.3, zorder=5)
    ax.add_patch(payload)
    ax.text(6.0, 3.4, "Láser+PSD", ha="center", va="center", fontsize=6.5)

    # Ventiladores (a ambos lados, apuntando al brazo/carga útil)
    for fx, fy, lbl in [(9.2, 3.2, "Ventilador 1"), (9.2, 1.6, "Ventilador 2"), (9.2, 4.6, "Ventilador 3")]:
        fan = Circle((fx, fy), 0.35, fc="#EAF7EA", ec=GREEN, lw=1.3, zorder=4)
        ax.add_patch(fan)
        for k in range(4):
            th = k * 90
            ax.plot([fx, fx + 0.3 * __import__("math").cos(__import__("math").radians(th))],
                    [fy, fy + 0.3 * __import__("math").sin(__import__("math").radians(th))],
                    color=GREEN, lw=1.0, zorder=5)
        ax.text(fx, fy - 0.6, lbl, ha="center", fontsize=6.5, color=GREEN)
        ax.annotate("", xy=(6.6, 3.4), xytext=(fx - 0.4, fy),
                    arrowprops=dict(arrowstyle="->", color=GREEN, lw=1.1, ls="dashed",
                                     connectionstyle="arc3,rad=0.15"))
    ax.text(9.2, 5.15, "ráfagas de viento\n(PWM variable)", ha="center", fontsize=6.8, color=GREEN)

    # Blanco fijo (target) para medir apuntamiento
    target_x = 0.3
    ax.plot([target_x, target_x], [1.2, 4.2], color="black", lw=2.5, zorder=3)
    for r, c in [(0.5, "#D62728"), (0.32, "white"), (0.15, "#D62728")]:
        ax.add_patch(Circle((target_x, 3.6), r, fc=c, ec="black", lw=0.8, zorder=4))
    ax.text(target_x, 4.5, "Blanco fijo\n(interior)", ha="center", fontsize=6.8)
    ax.annotate("", xy=(target_x + 0.5, 3.6), xytext=(6.5, 3.4),
                arrowprops=dict(arrowstyle="->", color=ACCENT, lw=1.2))
    ax.text(3.6, 4.05, "línea de visión (boresight)", ha="center", fontsize=6.8, color=ACCENT, rotation=-8)

    fig.tight_layout()
    save(fig, "fig12_banco_pruebas")


if __name__ == "__main__":
    print("Generando diagramas de diseño del prototipo físico ...")
    fig_block_diagram()
    fig_electrical_schematic()
    fig_mechanical_layout()
    fig_test_bench()
    print("Listo. Ver carpeta ../figures/")
