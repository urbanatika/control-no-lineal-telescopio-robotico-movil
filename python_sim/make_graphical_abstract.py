"""
Genera el "graphical abstract" requerido por el portal de envío de IEEE
Transactions on Control Systems Technology (TCST): una única imagen que
resume de un vistazo el problema, el método y el resultado principal del
artículo. Mismo estilo visual que make_diagrams.py / make_figures.py.

Salida -> ../figures/graphical_abstract.png (300 dpi) y .pdf
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

FIG_DIR = "../figures"

NAVY = "#1B2A4A"
GRAY = "#555555"
LIGHT = "#EEF1F8"
GOLD = "#F2A900"
GOLD_LIGHT = "#FFF3D6"

COLORS = {
    "Adaptativo-\nLyapunov": "#1f77b4",
    "SMC (Super-\nTwisting)": "#d62728",
    "MPC": "#2ca02c",
    "Neuro-Difuso\nAdaptativo": "#9467bd",
}
RMSE = {
    "Adaptativo-\nLyapunov": 191.7,
    "SMC (Super-\nTwisting)": 53.9,
    "MPC": 318.7,
    "Neuro-Difuso\nAdaptativo": 116.0,
}


def box(ax, x, y, w, h, text, fc=LIGHT, ec=NAVY, fontsize=9, textcolor="black", lw=1.3, zorder=3):
    b = FancyBboxPatch((x, y), w, h,
                        boxstyle="round,pad=0.02,rounding_size=0.08",
                        linewidth=lw, edgecolor=ec, facecolor=fc, zorder=zorder)
    ax.add_patch(b)
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center",
            fontsize=fontsize, color=textcolor, zorder=zorder + 1, linespacing=1.3)
    return (x, y, w, h)


def arrow(ax, p1, p2, color=GRAY, lw=1.6, rad=0.0):
    a = FancyArrowPatch(p1, p2, arrowstyle="-|>", mutation_scale=13,
                         linewidth=lw, color=color,
                         connectionstyle=f"arc3,rad={rad}", zorder=2)
    ax.add_patch(a)


def make_graphical_abstract():
    fig = plt.figure(figsize=(11, 4.6))
    gs = fig.add_gridspec(1, 2, width_ratios=[1.25, 1.0], wspace=0.06)
    axL = fig.add_subplot(gs[0, 0])
    axR = fig.add_subplot(gs[0, 1])

    # ---- Panel izquierdo: esquema del problema y del método ----------------
    axL.set_xlim(0, 10)
    axL.set_ylim(0, 8)
    axL.axis("off")
    axL.set_title("Telescopio robótico móvil bajo perturbación multi-evento",
                   fontsize=10.5, fontweight="bold", color=NAVY, pad=10)

    # Perturbaciones (izquierda)
    box(axL, 0.1, 6.1, 2.5, 1.1, "Sísmica\n(Coquimbo/Chiloé/\nMelipilla)", fc="#FCEBEA", fontsize=7.5)
    box(axL, 0.1, 4.5, 2.5, 1.1, "Viento\n(Gauss-Markov)", fc="#FCEBEA", fontsize=7.5)
    box(axL, 0.1, 2.9, 2.5, 1.1, "Microvibración\nde terreno (nuevo)", fc=GOLD_LIGHT, fontsize=7.5)

    # Planta: telescopio móvil 2 GDL
    box(axL, 3.1, 4.3, 2.6, 2.9, "Telescopio móvil\n2 GDL\n(acimut-elevación)\nbase con ruedas",
        fc=LIGHT, fontsize=8.5, lw=1.6)
    # flechas perturbaciones -> planta
    for y0 in (6.65, 5.05, 3.45):
        arrow(axL, (2.6, y0), (3.1, 5.75), rad=0.05 * (y0 - 5.05) / 1.6)

    # EKF
    box(axL, 6.1, 4.6, 2.1, 1.6, "EKF\nfusión\nsensorial\ncomún", fc="#EAF7EA", fontsize=8, lw=1.6)
    arrow(axL, (5.7, 5.75), (6.1, 5.4))

    # Cuatro leyes de control (recuadro contenedor + etiqueta arriba, sin solape)
    axL.text(7.55, 4.05, "4 leyes comparadas (mismo EKF)", ha="center", va="center",
              fontsize=7.3, color=GRAY, style="italic")
    ctrl_y = [2.35, 1.60, 0.85, 0.10]
    ctrl_labels = ["Adaptativo-Lyapunov", "SMC Super-Twisting", "MPC lineal", "Neuro-Difuso"]
    ctrl_colors = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd"]
    for y, lab, c in zip(ctrl_y, ctrl_labels, ctrl_colors):
        box(axL, 6.3, y, 2.5, 0.65, lab, fc="white", ec=c, fontsize=7.2, lw=1.5)
    arrow(axL, (7.55, 4.6), (7.55, 2.85), color=GRAY, lw=1.2)

    # ---- Panel derecho: resultado principal (RMSE comparado) ---------------
    labels = list(RMSE.keys())
    vals = [RMSE[l] for l in labels]
    bar_colors = [COLORS[l] for l in labels]
    bars = axR.bar(labels, vals, color=bar_colors, width=0.6, zorder=3)
    best_idx = vals.index(min(vals))
    bars[best_idx].set_edgecolor(GOLD)
    bars[best_idx].set_linewidth(3)

    for b, v in zip(bars, vals):
        axR.text(b.get_x() + b.get_width() / 2, v + 6, f"{v:.1f}\"",
                  ha="center", va="bottom", fontsize=9, fontweight="bold")

    axR.set_ylabel("RMSE de apuntamiento [arcsec]", fontsize=9)
    axR.set_title("Resultado: Modos Deslizantes logra\nel menor error de apuntamiento",
                   fontsize=10.5, fontweight="bold", color=NAVY, pad=10)
    axR.set_ylim(0, max(vals) * 1.22)
    axR.tick_params(axis="x", labelsize=7.8)
    axR.spines["top"].set_visible(False)
    axR.spines["right"].set_visible(False)
    axR.grid(axis="y", color="#dddddd", lw=0.6, zorder=0)

    fig.tight_layout()
    fig.savefig(f"{FIG_DIR}/graphical_abstract.png", dpi=300, bbox_inches="tight")
    fig.savefig(f"{FIG_DIR}/graphical_abstract.pdf", bbox_inches="tight")
    plt.close(fig)
    print("guardado graphical_abstract.png / .pdf")


if __name__ == "__main__":
    make_graphical_abstract()
