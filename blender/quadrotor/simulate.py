"""Quadrirotor en X piloté par une cascade de PID, simulation 6 DDL.

Structure de commande (proche de PX4) :

    position --PID--> accélération --> poussée + attitude voulue
    attitude --P----> vitesses angulaires de consigne
    vitesses angulaires --PID--> couples --mélangeur--> vitesses moteurs

Mission : décollage, huit en 3D, stationnaire sous rafale de vent latérale
(l'intégrateur ramène l'engin sur sa consigne), atterrissage. Aucune anticipation (feedforward)
d'accélération : le suivi repose sur le seul PID, d'où le retard visible
dans les virages.

Repère monde ENU (z vers le haut), repère engin FLU (x avant, y gauche,
z haut). Intégration d'Euler semi-implicite à 1 kHz, boucle de position à
100 Hz.

Usage :
    python simulate.py [--out sim.npz] [--fps 24] [--no-wind]
"""

import argparse

import numpy as np

# --- Engin (classe F450) ----------------------------------------------------
MASS = 1.2                                  # kg
G = 9.81                                    # m/s²
ARM = 0.225                                 # m, centre -> axe moteur
INERTIA = np.diag([0.011, 0.011, 0.021])    # kg·m²
KF = 1.0e-5                                 # N/(rad/s)², poussée d'une hélice
KM = 0.016 * KF                             # N·m/(rad/s)², couple de traînée
W_MAX = 1000.0                              # rad/s
TAU_MOTOR = 0.02                            # s, constante de temps moteur
DRAG = 0.3                                  # N/(m/s), traînée linéaire

# Moteurs : avant gauche, arrière gauche, arrière droit, avant droit.
# SPIN = +1 : hélice qui tourne dans le sens trigonométrique vu de dessus.
ROTOR_ANGLES = np.deg2rad([45.0, 135.0, 225.0, 315.0])
ROTOR_XY = ARM * np.column_stack([np.cos(ROTOR_ANGLES), np.sin(ROTOR_ANGLES)])
SPIN = np.array([1.0, -1.0, 1.0, -1.0])

# [T, τx, τy, τz] = MIX @ ω²
MIX = np.vstack([
    KF * np.ones(4),
    KF * ROTOR_XY[:, 1],
    -KF * ROTOR_XY[:, 0],
    -KM * SPIN,
])
MIX_INV = np.linalg.inv(MIX)

# --- Gains ------------------------------------------------------------------
KP_POS = np.array([6.0, 6.0, 8.0])          # 1/s²
KI_POS = np.array([2.5, 2.5, 3.0])          # 1/s³
KD_POS = np.array([4.5, 4.5, 5.0])          # 1/s
# Plan horizontal : s³ + Kd s² + Kp s + Ki = (s + 2,5)(s + 1)², pas de mode oscillant.
I_POS_MAX = np.array([3.0, 3.0, 4.0])       # m/s², anti-emballement
TILT_MAX = np.deg2rad(35.0)

K_ATT = np.array([7.0, 7.0, 3.0])           # 1/s
RATE_MAX = np.deg2rad(220.0)
KP_RATE = np.array([20.0, 20.0, 8.0])       # 1/s
KI_RATE = np.array([5.0, 5.0, 2.0])         # 1/s²
KD_RATE = np.array([0.15, 0.15, 0.0])       # sans unité
I_RATE_MAX = 20.0                           # rad/s²

# --- Mission ----------------------------------------------------------------
DT = 1e-3
POS_DECIMATION = 10                         # boucle position à 100 Hz
T_END = 29.0
T_ARM = 1.0
HOVER_Z = 1.5
TAKEOFF = (1.0, 4.0)
EIGHT = (5.0, 17.0)
EIGHT_RAMP = 2.0                            # s, mise en vitesse et arrêt du huit
LANDING = (24.5, 27.5)
EIGHT_A = 2.5                               # m, demi-largeur du huit
EIGHT_DZ = 0.4                              # m, ondulation verticale
GUST = (18.0, 22.0)
GUST_WIND = np.array([6.0, 0.0, 0.0])       # m/s


def min_jerk(tau):
    """Profil 5e ordre s(τ) et ds/dτ, nul en vitesse et accélération aux bouts."""
    tau = np.clip(tau, 0.0, 1.0)
    s = tau**3 * (10 - 15 * tau + 6 * tau**2)
    ds = 30 * tau**2 * (1 - tau) ** 2
    return s, ds


def _eight_phase():
    """Table θ(t), θ'(t) du huit : vitesse angulaire constante, rampes adoucies."""
    dur = EIGHT[1] - EIGHT[0]
    tt = np.linspace(0.0, dur, 4001)
    up, _ = min_jerk(tt / EIGHT_RAMP)
    down, _ = min_jerk((dur - tt) / EIGHT_RAMP)
    dth = up * down
    th = np.concatenate([[0.0], np.cumsum(0.5 * (dth[1:] + dth[:-1]) * np.diff(tt))])
    scale = 2 * np.pi / th[-1]
    return tt, th * scale, dth * scale


_EIGHT_T, _EIGHT_TH, _EIGHT_DTH = _eight_phase()


def reference(t):
    """Consigne de position et de vitesse à l'instant t."""
    p = np.zeros(3)
    v = np.zeros(3)
    if t < TAKEOFF[1]:
        dur = TAKEOFF[1] - TAKEOFF[0]
        s, ds = min_jerk((t - TAKEOFF[0]) / dur)
        p[2], v[2] = HOVER_Z * s, HOVER_Z * ds / dur
    elif t < LANDING[0]:
        te = np.clip(t - EIGHT[0], 0.0, _EIGHT_T[-1])
        th = np.interp(te, _EIGHT_T, _EIGHT_TH)
        dth = np.interp(te, _EIGHT_T, _EIGHT_DTH)
        p[:] = [EIGHT_A * np.sin(th), 0.5 * EIGHT_A * np.sin(2 * th),
                HOVER_Z + EIGHT_DZ * np.sin(th)]
        v[:] = [EIGHT_A * np.cos(th), EIGHT_A * np.cos(2 * th),
                EIGHT_DZ * np.cos(th)]
        v *= dth
    else:
        dur = LANDING[1] - LANDING[0]
        s, ds = min_jerk((t - LANDING[0]) / dur)
        # On vise légèrement sous le sol pour garantir le contact.
        p[2], v[2] = HOVER_Z - (HOVER_Z + 0.05) * s, -(HOVER_Z + 0.05) * ds / dur
    return p, v


def wind(t, enabled=True):
    """Rafale à fronts adoucis (0,5 s) pendant l'intervalle GUST."""
    if not enabled:
        return np.zeros(3)
    rise, _ = min_jerk((t - GUST[0]) / 0.5)
    fall, _ = min_jerk((t - GUST[1]) / 0.5)
    return GUST_WIND * (rise - fall)


def quat_to_rot(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def vee(m):
    return np.array([m[2, 1], m[0, 2], m[1, 0]])


def euler_zyx(rot):
    """Roulis, tangage, lacet (rad) d'une matrice de rotation."""
    roll = np.arctan2(rot[2, 1], rot[2, 2])
    pitch = -np.arcsin(np.clip(rot[2, 0], -1, 1))
    yaw = np.arctan2(rot[1, 0], rot[0, 0])
    return np.array([roll, pitch, yaw])


def simulate(wind_on=True):
    n = int(round(T_END / DT)) + 1
    pos = np.zeros(3)
    vel = np.zeros(3)
    quat = np.array([1.0, 0, 0, 0])
    rate = np.zeros(3)
    omega = np.zeros(4)
    prop_angle = np.zeros(4)

    i_pos = np.zeros(3)
    i_rate = np.zeros(3)
    rate_prev = np.zeros(3)
    d_rate = np.zeros(3)
    thrust_sp = 0.0
    rot_sp = np.eye(3)
    armed = False
    landed_at = None

    keys = ("t", "pos", "vel", "quat", "euler", "euler_sp", "rate", "omega",
            "prop_angle", "pos_ref", "vel_ref", "wind", "thrust")
    log = {k: [] for k in keys}

    for k in range(n):
        t = k * DT
        rot = quat_to_rot(quat)
        p_ref, v_ref = reference(t)
        on_ground = pos[2] <= 1e-6

        if not armed and t >= T_ARM and landed_at is None:
            armed = True
        if armed and t > LANDING[1] and on_ground:
            armed, landed_at = False, t

        # Boucle de position (100 Hz) : PID -> accélération voulue.
        if k % POS_DECIMATION == 0:
            e_p = p_ref - pos
            e_v = v_ref - vel
            if armed and not on_ground:
                i_pos = np.clip(i_pos + KI_POS * e_p * DT * POS_DECIMATION,
                                -I_POS_MAX, I_POS_MAX)
            acc = KP_POS * e_p + i_pos + KD_POS * e_v
            acc[2] = max(acc[2], -0.8 * G)
            acc_xy_max = (G + acc[2]) * np.tan(TILT_MAX)
            norm_xy = np.hypot(acc[0], acc[1])
            if norm_xy > acc_xy_max:
                acc[:2] *= acc_xy_max / norm_xy
            force = MASS * (acc + np.array([0, 0, G]))
            b3 = force / np.linalg.norm(force)
            b2 = np.cross(b3, [1.0, 0, 0])          # lacet de consigne nul
            b2 /= np.linalg.norm(b2)
            rot_sp = np.column_stack([np.cross(b2, b3), b2, b3])
            thrust_sp = force @ rot[:, 2]

        # Attitude (P) puis vitesses angulaires (PID), à 1 kHz.
        e_rot = 0.5 * vee(rot_sp.T @ rot - rot.T @ rot_sp)
        rate_sp = np.clip(-K_ATT * e_rot, -RATE_MAX, RATE_MAX)
        e_rate = rate_sp - rate
        if armed and not on_ground:
            i_rate = np.clip(i_rate + KI_RATE * e_rate * DT, -I_RATE_MAX, I_RATE_MAX)
        # Dérivée sur la mesure, filtrée au premier ordre (30 Hz).
        alpha_f = DT / (DT + 1 / (2 * np.pi * 30))
        d_rate += alpha_f * ((rate - rate_prev) / DT - d_rate)
        rate_prev = rate.copy()
        ang_acc = KP_RATE * e_rate + i_rate - KD_RATE * d_rate
        torque = INERTIA @ ang_acc + np.cross(rate, INERTIA @ rate)

        if armed:
            w2 = MIX_INV @ np.concatenate([[thrust_sp], torque])
            omega_cmd = np.sqrt(np.clip(w2, 0, W_MAX**2))
        else:
            omega_cmd = np.zeros(4)

        # Journal, avant intégration.
        log["t"].append(t)
        log["pos"].append(pos.copy())
        log["vel"].append(vel.copy())
        log["quat"].append(quat.copy())
        log["euler"].append(euler_zyx(rot))
        log["euler_sp"].append(euler_zyx(rot_sp))
        log["rate"].append(rate.copy())
        log["omega"].append(omega.copy())
        log["prop_angle"].append(prop_angle.copy())
        log["pos_ref"].append(p_ref)
        log["vel_ref"].append(v_ref)
        log["wind"].append(wind(t, wind_on))
        log["thrust"].append(KF * np.sum(omega**2))

        # Dynamique.
        omega += (omega_cmd - omega) * DT / TAU_MOTOR
        prop_angle += SPIN * omega * DT
        f_motors = KF * omega**2
        thrust = f_motors.sum()
        tau = np.array([
            f_motors @ ROTOR_XY[:, 1],
            -f_motors @ ROTOR_XY[:, 0],
            -KM * (SPIN @ omega**2),
        ])
        f_world = rot[:, 2] * thrust - DRAG * (vel - wind(t, wind_on))
        acc_w = f_world / MASS - np.array([0, 0, G])
        vel += acc_w * DT
        pos += vel * DT
        rate_dot = np.linalg.solve(INERTIA, tau - np.cross(rate, INERTIA @ rate))
        rate += rate_dot * DT
        quat = quat + 0.5 * quat_mul(quat, np.concatenate([[0.0], rate])) * DT
        quat /= np.linalg.norm(quat)

        # Sol : contact sans rebond, l'engin reste à plat.
        if pos[2] <= 0 and vel[2] <= 0:
            pos[2] = 0.0
            vel[:] = 0.0
            if thrust < MASS * G:
                rate[:] = 0.0
                yaw = euler_zyx(quat_to_rot(quat))[2]
                quat = np.array([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])

    return {k: np.asarray(v) for k, v in log.items()}


def resample(log, fps):
    """Échantillonne le journal à la cadence de l'animation."""
    t_frames = np.arange(0.0, T_END + 1e-9, 1.0 / fps)
    idx = np.clip(np.round(t_frames / DT).astype(int), 0, len(log["t"]) - 1)
    return {k: v[idx] for k, v in log.items()}


def summary(log):
    t = log["t"]
    err = np.linalg.norm(log["pos"] - log["pos_ref"], axis=1)
    eight = (t >= EIGHT[0]) & (t <= EIGHT[1])
    gust = (t >= GUST[0]) & (t <= GUST[1])
    hover = (t >= GUST[1] + 1.0) & (t <= LANDING[0])
    tilt = np.degrees(np.max(np.abs(log["euler"][:, :2])))
    return {
        "erreur max sur le huit (m)": err[eight].max(),
        "erreur RMS sur le huit (m)": np.sqrt(np.mean(err[eight] ** 2)),
        "erreur max pendant la rafale (m)": err[gust].max(),
        "erreur max après la rafale (m)": err[hover].max(),
        "inclinaison max (°)": tilt,
        "vitesse moteur max (rad/s)": log["omega"].max(),
        "fin de mission, z (m)": log["pos"][-1, 2],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", default="sim.npz")
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--no-wind", action="store_true")
    args = parser.parse_args()

    log = simulate(wind_on=not args.no_wind)
    frames = resample(log, args.fps)
    np.savez_compressed(
        args.out,
        **{f"full_{k}": v[::5] for k, v in log.items()},   # 200 Hz, pour les courbes
        **{f"frame_{k}": v for k, v in frames.items()},
        fps=args.fps, gust=np.array(GUST), rotor_xy=ROTOR_XY, spin=SPIN,
    )
    for name, value in summary(log).items():
        print(f"{name:40s} {value:8.3f}")
    print(f"-> {args.out} ({len(frames['t'])} images à {args.fps} i/s)")


if __name__ == "__main__":
    main()
