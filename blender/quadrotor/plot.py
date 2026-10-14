"""Courbes de la simulation : suivi de trajectoire, attitude, moteurs.

Usage :
    python plot.py sim.npz [--out courbes.png]
"""

import argparse

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#e4e3df"
REF = "#8a8984"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
GUST_FILL = "#ecebe7"

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
    "axes.edgecolor": GRID, "axes.labelcolor": INK_2, "axes.titlecolor": INK,
    "axes.titlesize": 11, "axes.titleweight": "bold", "axes.titlelocation": "left",
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
    "axes.spines.top": False, "axes.spines.right": False,
    "xtick.color": INK_2, "ytick.color": INK_2, "font.size": 9,
    "legend.frameon": False, "lines.linewidth": 1.8,
})


def shade_gust(ax, gust):
    ax.axvspan(*gust, color=GUST_FILL, zorder=0, lw=0)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("npz")
    parser.add_argument("--out", default="courbes.png")
    args = parser.parse_args()

    d = np.load(args.npz)
    t, pos, ref = d["full_t"], d["full_pos"], d["full_pos_ref"]
    gust = d["gust"]

    fig = plt.figure(figsize=(12, 10), layout="constrained")
    gs = fig.add_gridspec(3, 2, height_ratios=[1.3, 1, 1])
    fig.suptitle("Quadrirotor en X, cascade PID : décollage, huit 3D, "
                 "stationnaire sous rafale latérale de 6 m/s, atterrissage", color=INK, fontsize=13, x=0.01, ha="left")

    ax = fig.add_subplot(gs[0, 0])
    ax.plot(ref[:, 0], ref[:, 1], "--", color=REF, lw=1.4, label="consigne")
    ax.plot(pos[:, 0], pos[:, 1], color=SERIES[0], label="mesure")
    in_gust = (t >= gust[0]) & (t <= gust[1])
    ax.plot(pos[in_gust, 0], pos[in_gust, 1], color=SERIES[1], lw=2.4,
            label=f"pendant la rafale ({gust[0]:.0f}–{gust[1]:.0f} s)")
    ax.annotate("", xy=(-0.25, -0.9), xytext=(-1.05, -0.9),
                arrowprops=dict(arrowstyle="-|>", color=INK_2, lw=1.4))
    ax.text(-0.65, -0.82, "vent 6 m/s", color=INK_2, ha="center", fontsize=8)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Vue de dessus")
    ax.legend(loc="lower left", fontsize=8)

    ax = fig.add_subplot(gs[0, 1])
    shade_gust(ax, gust)
    for i, (name, color) in enumerate(zip("xyz", SERIES)):
        ax.plot(t, ref[:, i], "--", color=REF, lw=1.2)
        ax.plot(t, pos[:, i], color=color, label=name)
    ax.set_xlabel("t (s)")
    ax.set_ylabel("position (m)")
    ax.set_title("Position et consigne (tirets)")
    ax.legend(loc="upper right", ncol=3, fontsize=8)

    ax = fig.add_subplot(gs[1, 0])
    shade_gust(ax, gust)
    err = np.linalg.norm(pos - ref, axis=1)
    ax.plot(t, err, color=SERIES[0])
    ax.text(np.mean(gust), 0.97, "rafale", ha="center", va="top", color=INK_2,
            transform=ax.get_xaxis_transform())
    ax.set_xlabel("t (s)")
    ax.set_ylabel("‖p − p_ref‖ (m)")
    ax.set_title("Erreur de suivi")

    ax = fig.add_subplot(gs[1, 1])
    shade_gust(ax, gust)
    eul, eul_sp = np.degrees(d["full_euler"]), np.degrees(d["full_euler_sp"])
    for i, (name, color) in enumerate(zip(["roulis", "tangage"], SERIES)):
        ax.plot(t, eul_sp[:, i], "--", color=REF, lw=1.2)
        ax.plot(t, eul[:, i], color=color, label=name)
    ax.set_xlabel("t (s)")
    ax.set_ylabel("angle (°)")
    ax.set_title("Attitude et consigne (tirets)")
    ax.legend(loc="upper right", ncol=2, fontsize=8)

    ax = fig.add_subplot(gs[2, :])
    shade_gust(ax, gust)
    names = ["M1 avant gauche", "M2 arrière gauche", "M3 arrière droit", "M4 avant droit"]
    for i, (name, color) in enumerate(zip(names, SERIES)):
        ax.plot(t, d["full_omega"][:, i], color=color, lw=1.4, label=name)
    ax.set_xlabel("t (s)")
    ax.set_ylabel("ω (rad/s)")
    ax.set_title("Vitesses des moteurs")
    ax.legend(loc="lower center", ncol=4, fontsize=8)

    fig.savefig(args.out, dpi=130)
    print(f"-> {args.out}")


if __name__ == "__main__":
    main()
