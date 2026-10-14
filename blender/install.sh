#!/usr/bin/env bash
# Installe Blender (module bpy, PyPI) + le MCP communautaire mcp-for-blender
# dans un venv, puis lance le pont headless sur le port 9876.
set -euo pipefail

ENV_DIR="${BLENDER_MCP_ENV:-$HOME/blender/.venv}"
HERE="$(cd "$(dirname "$0")" && pwd)"

# bpy 5.2.x n'existe que pour CPython 3.13 ; uv le télécharge si besoin.
uv venv -p 3.13 "$ENV_DIR" --allow-existing -q
uv pip install -p "$ENV_DIR/bin/python" -q "bpy==5.2.2" "mcp-for-blender==2.1.9"
"$ENV_DIR/bin/python" -c "import bpy; print('Blender', bpy.app.version_string)"

# Au premier appel d'outil, le serveur MCP ouvre une demande de consentement à
# la télémétrie (elicitation). En session cloud personne n'y répond et l'appel
# expire au bout de 60 s. On enregistre un refus pour qu'il ne la pose plus.
"$ENV_DIR/bin/python" -c "from blender_mcp.consent_prompt import _write_state; \
_write_state(action='decline', consent=False, via='install.sh')"

if [[ "${1:-}" == "--start" ]]; then
  setsid nohup env DISABLE_TELEMETRY=1 "$ENV_DIR/bin/python" -u "$HERE/headless_bridge.py" \
    > "$(dirname "$ENV_DIR")/bridge.log" 2>&1 &
  echo "Pont lancé (PID $!), journal : $(dirname "$ENV_DIR")/bridge.log"
fi
