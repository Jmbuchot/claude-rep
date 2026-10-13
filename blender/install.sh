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

if [[ "${1:-}" == "--start" ]]; then
  nohup env DISABLE_TELEMETRY=1 "$ENV_DIR/bin/python" -u "$HERE/headless_bridge.py" \
    > "$(dirname "$ENV_DIR")/bridge.log" 2>&1 &
  echo "Pont lancé (PID $!), journal : $(dirname "$ENV_DIR")/bridge.log"
fi
