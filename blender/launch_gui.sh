#!/usr/bin/env bash
# Lance Blender graphique (paquet Ubuntu, 4.0.2) sur un écran virtuel Xvfb,
# avec l'add-on MCP for Blender activé : le serveur démarre sur le port 9876.
# Capture d'écran : DISPLAY=:99 import -window root capture.png
set -euo pipefail

ENV_DIR="${BLENDER_MCP_ENV:-$HOME/blender/.venv}"
ADDON="$("$ENV_DIR/bin/python" -c 'import blender_mcp.bundled.addon as a; print(a.__file__)')"
export DISPLAY="${DISPLAY_NUM:-:99}"

apt-get install -y -q blender python3-requests xvfb imagemagick xdotool >/dev/null
pgrep -x Xvfb >/dev/null || { Xvfb "$DISPLAY" -screen 0 1600x900x24 >/dev/null 2>&1 & sleep 2; }

cat > /tmp/enable_mcp_addon.py <<PY
import bpy, addon_utils
bpy.ops.preferences.addon_install(filepath="$ADDON", overwrite=True)
addon_utils.enable("addon", default_set=True, persistent=True)
bpy.ops.wm.save_userpref()
PY

nohup env DISABLE_TELEMETRY=1 blender --python /tmp/enable_mcp_addon.py \
  > "$(dirname "$ENV_DIR")/blender_gui.log" 2>&1 &
echo "Blender lancé (PID $!) sur $DISPLAY"
