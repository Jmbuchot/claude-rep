"""Incrustation (HUD) : temps, phase de vol, erreur, mini-courbe et légende.

Produit une image PNG transparente par image de l'animation, à superposer
au rendu Blender avec ffmpeg (voir make_video.sh).

Usage :
    python hud.py sim.npz hud/ [--taille 960x540]
"""

import argparse
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D

import simulate as sim

INK = "#0b0b0b"
INK_2 = "#52514e"
PANEL = (0.99, 0.99, 0.98, 0.82)
BLUE, ORANGE, GREEN, GREY = "#2a78d6", "#eb6834", "#1baf7a", "#8a8984"
GUST_FILL = "#e3e2dd"

PHASES = [
    (0.0, "Au sol, moteurs arrêtés"),
    (sim.TAKEOFF[0], "Décollage"),
    (sim.TAKEOFF[1], "Stationnaire"),
    (sim.EIGHT[0], "Huit 3D : PID seul, sans anticipation"),
    (sim.EIGHT[1], "Stationnaire"),
    (sim.GUST[0], "Rafale latérale de 6 m/s"),
    (sim.GUST[1], "Fin de rafale : l'intégrale se vide"),
    (sim.LANDING[0], "Atterrissage"),
    (sim.LANDING[1], "Posé, moteurs coupés"),
]


def phase(t):
    return [name for start, name in PHASES if t >= start][-1]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("npz")
    parser.add_argument("dossier")
    parser.add_argument("--taille", default="960x540")
    args = parser.parse_args()
    width, height = (int(v) for v in args.taille.split("x"))
    os.makedirs(args.dossier, exist_ok=True)

    d = np.load(args.npz)
    t_frames = d["frame_t"]
    err_frames = np.linalg.norm(d["frame_pos"] - d["frame_pos_ref"], axis=1)
    t_full = d["full_t"]
    err_full = np.linalg.norm(d["full_pos"] - d["full_pos_ref"], axis=1)
    tilt = np.degrees(np.arccos(np.clip(
        np.cos(d["frame_euler"][:, 0]) * np.cos(d["frame_euler"][:, 1]), -1, 1)))

    dpi = 100
    fig = plt.figure(figsize=(width / dpi, height / dpi), dpi=dpi)
    fig.patch.set_alpha(0.0)
    s = width / 960                                   # échelle des polices

    # Bandeau haut gauche : temps, phase, erreur.
    head = fig.text(0.015, 0.975, "", ha="left", va="top", color=INK,
                    fontsize=13 * s, fontweight="bold",
                    bbox=dict(boxstyle="round,pad=0.45", fc=PANEL, ec="none"))
    sub = fig.text(0.015, 0.885, "", ha="left", va="top", color=INK_2,
                   fontsize=10 * s, family="monospace",
                   bbox=dict(boxstyle="round,pad=0.4", fc=PANEL, ec="none"))

    # Légende haut droite.
    handles = [
        Line2D([], [], color=GREY, lw=2, label="trajectoire de consigne"),
        Line2D([], [], color=BLUE, lw=3, label="trajectoire réelle"),
        Line2D([], [], color=GREEN, marker="o", lw=0, ms=7, label="point de consigne"),
        Line2D([], [], color=ORANGE, lw=2.5, label="vecteur erreur"),
    ]
    leg = fig.legend(handles=handles, loc="upper right", bbox_to_anchor=(0.99, 0.985),
                     fontsize=8.5 * s, frameon=True, framealpha=0.82,
                     facecolor=PANEL[:3], edgecolor="none", labelcolor=INK_2)
    leg.get_frame().set_boxstyle("round,pad=0.3")

    # Mini-courbe bas droite : erreur de suivi.
    ax = fig.add_axes([0.655, 0.095, 0.33, 0.22])
    ax.set_facecolor(PANEL)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color("#c9c8c3")
    ax.axvspan(*sim.GUST, color=GUST_FILL, lw=0)
    ax.text(np.mean(sim.GUST), 0.97, "rafale", transform=ax.get_xaxis_transform(),
            ha="center", va="top", fontsize=7.5 * s, color=INK_2)
    ax.plot(t_full, err_full, color="#d4d3ce", lw=1.2)        # courbe complète, en fond
    line, = ax.plot([], [], color=BLUE, lw=1.8)
    dot, = ax.plot([], [], "o", color=BLUE, ms=4)
    ax.set_xlim(0, t_full[-1])
    ax.set_ylim(0, err_full.max() * 1.15)
    ax.set_title("‖p − p_ref‖ (m)", fontsize=8.5 * s, color=INK_2, loc="left", pad=3)
    ax.tick_params(labelsize=7.5 * s, colors=INK_2, length=2)
    ax.set_xlabel("t (s)", fontsize=7.5 * s, color=INK_2, labelpad=1)

    for i, t in enumerate(t_frames):
        head.set_text(f"t = {t:5.2f} s   {phase(t)}")
        sub.set_text(f"erreur {err_frames[i]:5.2f} m   inclinaison {tilt[i]:4.1f}°")
        k = np.searchsorted(t_full, t, side="right")
        line.set_data(t_full[:k], err_full[:k])
        dot.set_data([t], [err_frames[i]])
        # transparent=True effacerait aussi le fond du panneau de la courbe.
        fig.savefig(os.path.join(args.dossier, f"{i + 1:04d}.png"), facecolor=(0, 0, 0, 0))
    print(f"-> {len(t_frames)} images dans {args.dossier}")


if __name__ == "__main__":
    main()
