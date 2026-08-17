"""
Genera todas las figuras comparativas y la tabla de métricas a partir de
sim_results.pkl (producido por main_simulation.py). Salidas -> ../figures/
como PNG (300dpi) y PDF vectorial, listas para el informe LaTeX.
"""
import pickle
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker

FIG_DIR = "../figures"
DT = 1e-3

COLORS = {
    "Adaptativo-Lyapunov+EKF": "#1f77b4",
    "SMC (Super-Twisting)": "#d62728",
    "MPC": "#2ca02c",
    "Neuro-Difuso Adaptativo": "#9467bd",
}
PHASES = [("Basal", 0, 5), ("Coquimbo 2015", 5, 25), ("Chiloé 2016", 25, 45), ("Melipilla 2017", 45, 60)]


def shade_phases(ax):
    shades = ["#f0f0f0", "#ffe0e0", "#e0eaff", "#fff2cc"]
    for (name, t0, t1), c in zip(PHASES, shades):
        ax.axvspan(t0, t1, color=c, alpha=0.5, zorder=0)


def save(fig, name):
    fig.savefig(f"{FIG_DIR}/{name}.png", dpi=300, bbox_inches="tight")
    fig.savefig(f"{FIG_DIR}/{name}.pdf", bbox_inches="tight")
    plt.close(fig)
    print(f"  guardado {name}.png / .pdf")


def fig_error_comparison(logs):
    fig, ax = plt.subplots(figsize=(9, 4.5))
    shade_phases(ax)
    for label, log in logs.items():
        ax.plot(log["t"], log["err_total_arcsec"], label=label, color=COLORS[label], lw=0.8)
    ax.set_yscale("log")
    ax.set_xlabel("Tiempo [s]")
    ax.set_ylabel("Error de apuntamiento total [arcsec] (escala log)")
    ax.set_title("Error de apuntamiento comparado — 4 leyes de control (60 s, multi-evento)")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, 1.22), fontsize=8, ncol=4, frameon=False)
    ax.set_xlim(0, 60)
    for name, t0, t1 in PHASES:
        ax.text((t0 + t1) / 2, ax.get_ylim()[1] * 0.55, name, ha="center", fontsize=7, color="gray")
    fig.tight_layout()
    save(fig, "fig1_error_comparativo")


def fig_control_effort(logs):
    fig, axs = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    for label, log in logs.items():
        axs[0].plot(log["t"], log["tau1"], label=label, color=COLORS[label], lw=0.6)
        axs[1].plot(log["t"], log["tau2"], label=label, color=COLORS[label], lw=0.6)
    for ax, name in zip(axs, ["Acimut (tau1)", "Elevación (tau2)"]):
        shade_phases(ax)
        ax.set_ylabel(f"Par [N·m]\n{name}")
        ax.set_xlim(0, 60)
    axs[0].set_title("Esfuerzo de control comparado")
    axs[0].legend(loc="upper right", fontsize=8, ncol=2)
    axs[1].set_xlabel("Tiempo [s]")
    fig.tight_layout()
    save(fig, "fig2_esfuerzo_control")


def fig_ekf_estimation(logs):
    log = logs["Adaptativo-Lyapunov+EKF"]
    fig, axs = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    axs[0].plot(log["t"], log["tv1_true"], label="Viento real (τv1)", lw=0.6, color="black", alpha=0.6)
    axs[0].plot(log["t"], log["tv1_hat"], label="Viento estimado EKF (τ̂v1)", lw=0.8, color="#d62728")
    axs[1].plot(log["t"], log["tv2_true"], label="Viento real (τv2)", lw=0.6, color="black", alpha=0.6)
    axs[1].plot(log["t"], log["tv2_hat"], label="Viento estimado EKF (τ̂v2)", lw=0.8, color="#1f77b4")
    for ax, name in zip(axs, ["Acimut", "Elevación"]):
        shade_phases(ax)
        ax.set_ylabel(f"Par de viento [N·m]\n{name}")
        ax.legend(loc="upper right", fontsize=8)
        ax.set_xlim(0, 60)
    axs[0].set_title("Estimación del torque de viento por el EKF (proceso Gauss-Markov, ec. 4)")
    axs[1].set_xlabel("Tiempo [s]")
    fig.tight_layout()
    save(fig, "fig3_ekf_estimacion_viento")

    rmse1 = np.sqrt(np.mean((log["tv1_true"] - log["tv1_hat"]) ** 2))
    rmse2 = np.sqrt(np.mean((log["tv2_true"] - log["tv2_hat"]) ** 2))
    return rmse1, rmse2


def fig_theta_convergence(logs):
    log = logs["Adaptativo-Lyapunov+EKF"]
    theta1, theta2 = log["theta1"], log["theta2"]
    t = log["t"]
    true1 = [3.2, 0.55]
    true2 = [1.75, 0.40, 3 * 9.81 * 0.15]
    labels1 = ["J1 [kg·m²]", "b1 [N·m·s/rad]"]
    labels2 = ["J2 [kg·m²]", "b2 [N·m·s/rad]", "mgl [N·m]"]

    fig, axs = plt.subplots(1, 5, figsize=(16, 3.2))
    for i in range(2):
        axs[i].plot(t, theta1[:, i], color="#1f77b4")
        axs[i].axhline(true1[i], color="black", ls="--", lw=0.8, label="valor real")
        axs[i].set_title(labels1[i])
        axs[i].set_xlabel("t [s]")
        axs[i].legend(fontsize=7)
    for i in range(3):
        axs[2 + i].plot(t, theta2[:, i], color="#d62728")
        axs[2 + i].axhline(true2[i], color="black", ls="--", lw=0.8, label="valor real")
        axs[2 + i].set_title(labels2[i])
        axs[2 + i].set_xlabel("t [s]")
        axs[2 + i].legend(fontsize=7)
    fig.suptitle("Convergencia de parámetros estimados θ̂(t) — Ley Adaptativa de Lyapunov (ec. 12)")
    fig.tight_layout()
    save(fig, "fig4_convergencia_parametrica")


def fig_sliding_surface(logs):
    log = logs["SMC (Super-Twisting)"]
    fig, axs = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
    axs[0].plot(log["t"], log["s1"], color="#d62728", lw=0.6)
    axs[1].plot(log["t"], log["s2"], color="#d62728", lw=0.6)
    for ax, name in zip(axs, ["Acimut", "Elevación"]):
        shade_phases(ax)
        ax.axhline(0, color="black", lw=0.5)
        ax.set_ylabel(f"s(t) [rad/s]\n{name}")
        ax.set_xlim(0, 60)
    axs[0].set_title("Superficie deslizante s(t) — SMC Super-Twisting")
    axs[1].set_xlabel("Tiempo [s]")
    fig.tight_layout()
    save(fig, "fig5_superficie_deslizante")


def fig_compute_time(metrics):
    labels = [m["label"] for m in metrics]
    times = [m["compute_ms"] for m in metrics]
    fig, ax = plt.subplots(figsize=(7, 4.2))
    bars = ax.bar(labels, times, color=[COLORS[l] for l in labels])
    ax.set_ylabel("Tiempo de cómputo promedio por paso [ms]")
    ax.set_title("Viabilidad computacional en tiempo real\n(lazo objetivo: 1 kHz → 1 ms)")
    ax.axhline(1.0, color="red", ls="--", lw=1, label="Límite lazo 1 kHz (1 ms)")
    ax.legend(fontsize=8)
    ax.tick_params(axis="x", rotation=20)
    for b, v in zip(bars, times):
        ax.text(b.get_x() + b.get_width() / 2, v, f"{v:.3f}", ha="center", va="bottom", fontsize=8)
    fig.tight_layout()
    save(fig, "fig6_tiempo_computo")


def fig_rmse_bar(metrics):
    labels = [m["label"] for m in metrics]
    phases = ["Basal", "Coquimbo", "Chiloe", "Melipilla"]
    x = np.arange(len(phases))
    width = 0.2
    fig, ax = plt.subplots(figsize=(9, 4.5))
    for i, m in enumerate(metrics):
        vals = [m[f"rmse_{p}"] for p in phases]
        ax.bar(x + (i - 1.5) * width, vals, width, label=m["label"], color=COLORS[m["label"]])
    ax.set_xticks(x)
    ax.set_xticklabels(["Basal", "Coquimbo 2015", "Chiloé 2016", "Melipilla 2017"])
    ax.set_ylabel("RMSE de apuntamiento por fase [arcsec]")
    ax.set_title("RMSE comparado por fase sísmica")
    ax.legend(fontsize=8)
    fig.tight_layout()
    save(fig, "fig7_rmse_por_fase")


def fig_chattering(metrics):
    labels = [m["label"] for m in metrics]
    vals = [m["chattering"] for m in metrics]
    fig, ax = plt.subplots(figsize=(6, 4))
    bars = ax.bar(labels, vals, color=[COLORS[l] for l in labels])
    ax.set_ylabel("Índice de chattering (variación total del par) [N·m]")
    ax.set_title("Suavidad de la señal de control (menor = más suave)")
    ax.tick_params(axis="x", rotation=20)
    fig.tight_layout()
    save(fig, "fig8_chattering")


def write_metrics_table(metrics, rmse_wind):
    lines = []
    lines.append("| Controlador | RMSE total [arcsec] | Max total [arcsec] | Esfuerzo (∫τ²dt) | Chattering [N·m] | Cómputo/paso [ms] |")
    lines.append("|---|---|---|---|---|---|")
    for m in metrics:
        lines.append(f"| {m['label']} | {m['rmse_total']:.1f} | {m['max_total']:.1f} | "
                      f"{m['effort']:.1f} | {m['chattering']:.1f} | {m['compute_ms']:.4f} |")
    lines.append("")
    lines.append(f"RMSE de estimación de viento del EKF: eje acimut = {rmse_wind[0]:.4f} N·m, "
                 f"eje elevación = {rmse_wind[1]:.4f} N·m.")
    with open(f"{FIG_DIR}/tabla_metricas.md", "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print("  guardado tabla_metricas.md")

    # También como filas .tex para incluir directamente en el informe
    tex_lines = []
    for m in metrics:
        tex_lines.append(f"{m['label']} & {m['rmse_total']:.1f} & {m['max_total']:.1f} & "
                          f"{m['effort']:.0f} & {m['chattering']:.0f} & {m['compute_ms']:.4f} \\\\")
    with open(f"{FIG_DIR}/tabla_metricas.tex", "w", encoding="utf-8") as f:
        f.write("\n".join(tex_lines))
    print("  guardado tabla_metricas.tex")


if __name__ == "__main__":
    print("Cargando sim_results.pkl ...")
    with open("sim_results.pkl", "rb") as f:
        data = pickle.load(f)
    logs, metrics = data["logs"], data["metrics"]

    print("Generando figuras ...")
    fig_error_comparison(logs)
    fig_control_effort(logs)
    rmse_wind = fig_ekf_estimation(logs)
    fig_theta_convergence(logs)
    fig_sliding_surface(logs)
    fig_compute_time(metrics)
    fig_rmse_bar(metrics)
    fig_chattering(metrics)
    write_metrics_table(metrics, rmse_wind)
    print("Listo. Ver carpeta ../figures/")
