# Quadrirotor en X piloté par PID, simulé en Python, rendu dans Blender

Une simulation 6 DDL d'un quadrirotor de classe F450 commandé par une
cascade de PID, rejouée image par image dans Blender 5.2 et rendue avec
Cycles.

```
simulate.py ──sim.npz──┬─> plot.py ──────────> courbes.png
                       ├─> build_scene.py ───> quadrirotor.blend ──render.py──> images/
                       └─> hud.py ───────────> hud/ ──ffmpeg (incrustation)──> quadrirotor_pid.mp4
```

`./make_video.sh [dossier]` enchaîne tout. Le rendu prend environ 1 h 30
sur 4 cœurs (697 images 960×540, 20 échantillons et débruitage OIDN).

## Modèle

| Grandeur | Valeur |
|---|---|
| masse | 1,2 kg |
| bras (centre → moteur) | 0,225 m |
| inerties Jxx, Jyy, Jzz | 0,011 ; 0,011 ; 0,021 kg·m² |
| poussée d'une hélice | kf ω², kf = 1,0·10⁻⁵ N/(rad/s)² |
| couple de traînée | km = 0,016 kf |
| moteur | premier ordre, τ = 20 ms, ω ≤ 1000 rad/s |
| traînée de l'air | linéaire, 0,3 N/(m/s), sur la vitesse relative au vent |

Repère monde ENU, repère engin FLU. Rotation par quaternion, intégration
d'Euler semi-implicite à 1 kHz. Contact au sol sans rebond.

## Commande

```
p_ref ─>(PID position, 100 Hz)─> a_cmd ─> poussée T + attitude R_d
R_d ───>(P attitude)───────────> Ω_ref
Ω_ref ─>(PID vitesses angulaires, 1 kHz)─> couples ─> mélangeur X ─> ω₁..ω₄
```

- **Position** : `a = Kp e + Ki ∫e + Kd ė`, sans anticipation d'accélération.
  Plan horizontal : Kp = 6, Kd = 4,5, Ki = 2,5, soit
  `s³ + Kd s² + Kp s + Ki = (s + 2,5)(s + 1)²` (aucun mode oscillant).
  Vertical : Kp = 8, Kd = 5, Ki = 3. Anti-emballement par saturation de
  l'intégrale, inclinaison limitée à 35°.
- **Poussée et attitude** : la force voulue `m (a + g e_z)` donne l'axe z
  de l'engin voulu (lacet nul) ; la poussée est sa projection sur l'axe z
  courant.
- **Attitude** : erreur sur SO(3), `e_R = ½ (R_dᵀR − RᵀR_d)^∨`, puis
  `Ω_ref = −K_att e_R` (K_att = 7 ; 7 ; 3 s⁻¹).
- **Vitesses angulaires** : PID avec dérivée sur la mesure filtrée à 30 Hz
  (Kp = 20 ; 20 ; 8, Ki = 5 ; 5 ; 2), compensation gyroscopique `Ω × JΩ`.
- **Mélangeur** : inversion de la matrice 4×4 [T, τx, τy, τz] = M ω².

## Mission et résultats

| Phase | t (s) |
|---|---|
| armement | 1 |
| décollage à 1,5 m (profil 5ᵉ ordre) | 1 → 4 |
| huit 3D de 5 m × 2,5 m, ±0,4 m en altitude, ~2 m/s | 5 → 17 |
| stationnaire, rafale latérale de 6 m/s (+x) | 18 → 22 |
| atterrissage, moteurs coupés au contact | 24,5 → 27,5 |

Sortie de `simulate.py` :

```
erreur max sur le huit (m)                  0.386
erreur RMS sur le huit (m)                  0.260
erreur max pendant la rafale (m)            0.189
erreur max après la rafale (m)              0.181
inclinaison max (°)                        15.457
vitesse moteur max (rad/s)                574.946
```

Sur le huit, le PID seul traîne derrière la consigne : l'erreur vient de
l'accélération de référence que rien n'anticipe. Pendant la rafale, l'engin
part de 0,19 m puis l'intégrale le ramène. Quand le vent tombe, elle
contient encore la compensation et l'écarte de l'autre côté. Ajouter
`a_ref` en anticipation ramènerait l'erreur sur le huit à quelques
centimètres.

## Scène Blender

- Drone : plaque carbone, bras avant rouges et arrière blancs (le cap se lit
  à l'image), moteurs à cloche bleue, hélices bipales 10", train
  d'atterrissage, LED avant vertes et arrière rouges.
- Les hélices tournent à leur vitesse simulée (~550 rad/s). Le flou de
  mouvement de Cycles (obturateur 0,5 image, 32 poses par image pour les
  hélices) les transforme en disque translucide.
- Sphère verte : point de consigne. Trait orange : vecteur erreur. Tube gris :
  trajectoire de consigne. Tube bleu : trajectoire réelle, qui se dessine au
  fil du temps. Flèches bleues : vent, à l'échelle de la rafale.
- La scène est décalée de 0,115 m vers le haut pour que les pieds touchent le
  sol quand le centre de gravité simulé est à z = 0.
- Caméra de poursuite à distance à peu près constante, visée lissée par un
  filtre aller-retour (sans retard).

`build_scene.py` peut tourner dans le Blender piloté par le MCP :

```python
import sys; sys.path.insert(0, "blender/quadrotor")
import build_scene; build_scene.build("sim.npz", "quadrirotor.blend")
```

Le rendu de l'animation passe par `render.py`, dans un processus à part, pour
ne pas bloquer le pont MCP pendant une heure.
