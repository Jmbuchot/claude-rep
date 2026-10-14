"""Son des moteurs synthétisé à partir des vitesses de rotation simulées.

Pour chaque moteur i, à partir de ω_i(t) :
- raie de passage des pales (BPF = 2 pales × ω/2π, ~175 Hz en stationnaire)
  et ses harmoniques, plus une faible raie à la fréquence de rotation
  (balourd) ;
- sifflement électrique du moteur sans balais, 7 paires de pôles
  (7 × ω/2π, ~610 Hz) ;
- souffle aérodynamique large bande.
Les phases sont intégrées (φ = ∫ω dt) : les quatre moteurs, à des vitesses
légèrement différentes, battent entre eux comme sur un vrai drone.
L'amplitude tonale suit (ω/ω_stationnaire)^2,5 et le souffle la puissance 3
(la puissance acoustique d'une hélice croît comme une puissance élevée de la
vitesse en bout de pale).

Spatialisation : atténuation en 1/r par rapport à la caméra, panoramique
stéréo selon la position du drone dans le cadre. Le vent de la rafale
ajoute un souffle grave non spatialisé.

Usage :
    python sound.py sim.npz son.wav [--spectre spectre.png]
"""

import argparse
import wave

import numpy as np

from build_scene import Z_OFF, camera_path

SR = 48000
BLADES = 2
POLE_PAIRS = 7
OMEGA_HOVER = 545.0           # rad/s, vitesse moteur en stationnaire
HALF_FOV_TAN = 18.0 / 50.0    # capteur 36 mm, focale 50 mm
RNG = np.random.default_rng(7)


def band_noise(n, f_lo, f_hi, tilt=0.5):
    """Bruit blanc filtré dans le domaine fréquentiel, pente en 1/f^tilt."""
    spec = np.fft.rfft(RNG.standard_normal(n))
    f = np.fft.rfftfreq(n, 1 / SR)
    shape = np.zeros_like(f)
    band = (f >= f_lo) & (f <= f_hi)
    shape[band] = (f[band] / f_lo) ** -tilt
    x = np.fft.irfft(spec * shape, n)
    return x / np.std(x)


def slow_noise(n, cutoff, depth):
    """Modulation lente (turbulence) : 1 + depth × bruit sous `cutoff` Hz."""
    x = band_noise(n, 0.3, cutoff, tilt=0.0)
    return 1.0 + depth * x / np.max(np.abs(x))


def synth(d):
    t_full = d["full_t"]
    n = int(t_full[-1] * SR) + 1
    t = np.arange(n) / SR
    omega = np.column_stack([np.interp(t, t_full, d["full_omega"][:, i]) for i in range(4)])

    drone = np.zeros(n)
    for i in range(4):
        w = omega[:, i]
        phi = np.cumsum(w) / SR                      # angle de l'hélice (rad)
        gain = (w / OMEGA_HOVER) ** 2.5 * slow_noise(n, 12.0, 0.12)
        tone = sum(k ** -1.1 * np.sin(BLADES * k * phi + RNG.uniform(0, 2 * np.pi))
                   for k in range(1, 11))
        tone += 0.15 * np.sin(phi)                   # balourd, 1 × rotation
        whine = 0.10 * np.sin(POLE_PAIRS * phi) + 0.04 * np.sin(2 * POLE_PAIRS * phi)
        drone += gain * (tone + whine)
    broadband = np.mean((omega / OMEGA_HOVER) ** 3, axis=1)
    drone += 0.45 * broadband * band_noise(n, 150.0, 7000.0)

    # Spatialisation par rapport à la caméra du rendu.
    fps = int(d["fps"])
    pos_f = d["frame_pos"] + np.array([0, 0, Z_OFF])
    aim_f, cam_f = camera_path(pos_f, fps)
    t_f = d["frame_t"]
    pos, aim, cam = (np.column_stack([np.interp(t, t_f, a[:, k]) for k in range(3)])
                     for a in (pos_f, aim_f, cam_f))
    rel = pos - cam
    dist = np.linalg.norm(rel, axis=1)
    fwd = (aim - cam) / np.linalg.norm(aim - cam, axis=1, keepdims=True)
    right = np.cross(fwd, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right, axis=1, keepdims=True)
    azim = np.sum(rel * right, axis=1) / np.maximum(np.sum(rel * fwd, axis=1), 0.1)
    pan = 0.7 * np.clip(azim / HALF_FOV_TAN, -1, 1)
    drone *= np.median(dist) / dist
    left = drone * np.cos((pan + 1) * np.pi / 4)
    right_ch = drone * np.sin((pan + 1) * np.pi / 4)

    # Souffle du vent pendant la rafale, centré.
    wind = np.interp(t, t_full, np.linalg.norm(d["full_wind"], axis=1)) / 6.0
    gusts = wind * slow_noise(n, 1.5, 0.5)
    left += 0.6 * gusts * band_noise(n, 60.0, 900.0, tilt=1.0)
    right_ch += 0.6 * gusts * band_noise(n, 60.0, 900.0, tilt=1.0)

    stereo = np.column_stack([left, right_ch])
    # Fondu de 20 ms aux extrémités, crête à −1 dBFS.
    fade = np.minimum(1.0, np.minimum(t, t[-1] - t) / 0.02)
    stereo *= fade[:, None]
    stereo *= 10 ** (-1 / 20) / np.max(np.abs(stereo))
    return stereo, omega


def write_wav(path, stereo):
    pcm = np.round(stereo * 32767).astype("<i2")
    with wave.open(path, "wb") as f:
        f.setnchannels(2)
        f.setsampwidth(2)
        f.setframerate(SR)
        f.writeframes(pcm.tobytes())


def plot_spectrum(path, stereo, omega, gust):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    mono = stereo.mean(axis=1)
    fig, ax = plt.subplots(figsize=(11, 4.5), layout="constrained")
    fig.patch.set_facecolor("#fcfcfb")
    ax.specgram(mono, NFFT=8192, Fs=SR, noverlap=6144, cmap="magma",
                vmin=-110, vmax=-20)
    t = np.arange(len(mono)) / SR
    step = SR // 50
    bpf = BLADES * omega[::step].mean(axis=1) / (2 * np.pi)
    ax.plot(t[::step], bpf, "--", color="#86b6ef", lw=1, label="BPF = 2 ω/2π (moyenne)")
    ax.plot(t[::step], POLE_PAIRS * bpf / BLADES, ":", color="#86b6ef", lw=1,
            label="sifflement moteur = 7 ω/2π")
    ax.axvspan(*gust, color="white", alpha=0.08)
    ax.set_ylim(0, 2000)
    ax.set_xlabel("t (s)")
    ax.set_ylabel("fréquence (Hz)")
    ax.set_title("Spectrogramme du son des moteurs", loc="left", fontweight="bold")
    ax.legend(loc="upper right", fontsize=8, facecolor="#1a1a19", labelcolor="#f0efec",
              edgecolor="none")
    fig.savefig(path, dpi=120)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("npz")
    parser.add_argument("wav")
    parser.add_argument("--spectre", help="enregistre aussi un spectrogramme (PNG)")
    args = parser.parse_args()

    d = np.load(args.npz)
    stereo, omega = synth(d)
    write_wav(args.wav, stereo)
    print(f"-> {args.wav} ({len(stereo) / SR:.1f} s, {SR} Hz, stéréo)")
    if args.spectre:
        plot_spectrum(args.spectre, stereo, omega, d["gust"])
        print(f"-> {args.spectre}")


if __name__ == "__main__":
    main()
