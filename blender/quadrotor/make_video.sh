#!/usr/bin/env bash
# Chaîne complète : simulation -> courbes -> scène Blender -> rendu -> HUD -> son -> MP4.
# Usage : ./make_video.sh [dossier_sortie]   (par défaut ./sortie)
# Rendu Cycles CPU : ~16 s par image et par processus à 960x540 sur 2 cœurs.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="$(mkdir -p "${1:-$HERE/sortie}" && cd "${1:-$HERE/sortie}" && pwd)"
PY="${BLENDER_MCP_ENV:-$HOME/blender/.venv}/bin/python"
JOBS="${JOBS:-2}"

cd "$HERE"
"$PY" simulate.py --out "$OUT/sim.npz"
"$PY" plot.py "$OUT/sim.npz" --out "$OUT/courbes.png"
"$PY" build_scene.py "$OUT/sim.npz" "$OUT/quadrirotor.blend"

N=$("$PY" -c "import numpy as np; print(len(np.load('$OUT/sim.npz')['frame_t']))")
CORES=$(nproc)
for ((j = 0; j < JOBS; j++)); do
  "$PY" render.py "$OUT/quadrirotor.blend" "$OUT/images" \
    --debut $((j * N / JOBS + 1)) --fin $(((j + 1) * N / JOBS)) \
    --threads $((CORES / JOBS)) > "$OUT/rendu_$j.log" 2>&1 &
done
wait

"$PY" hud.py "$OUT/sim.npz" "$OUT/hud"
"$PY" sound.py "$OUT/sim.npz" "$OUT/son.wav" --spectre "$OUT/spectre.png"
ffmpeg -loglevel error -y -framerate 24 -i "$OUT/images/%04d.png" \
  -framerate 24 -i "$OUT/hud/%04d.png" -i "$OUT/son.wav" \
  -filter_complex "[0][1]overlay=format=auto,format=yuv420p[v]" \
  -map "[v]" -map 2:a -c:v libx264 -crf 18 -preset slow \
  -c:a aac -b:a 192k -shortest -movflags +faststart "$OUT/quadrirotor_pid.mp4"
echo "-> $OUT/quadrirotor_pid.mp4"
