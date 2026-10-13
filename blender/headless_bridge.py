"""Pont MCP-for-Blender sans interface graphique.

L'add-on communautaire (ahujasid/mcp-for-blender) refuse de démarrer quand
bpy.app.background est vrai, car en mode `blender -b` les timers de Blender ne
tournent pas et les commandes ne seraient jamais exécutées. Ici on charge
Blender comme module Python (paquet PyPI `bpy`), on enregistre l'add-on livré
avec le paquet `mcp-for-blender`, puis on ouvre le socket nous-mêmes et on
vide la file de commandes dans une boucle sur le thread principal, ce qui
remplace le timer de l'interface.

Usage :
    .venv/bin/python headless_bridge.py [--port 9876] [--blend scene.blend]
"""

import argparse
import signal
import socket
import threading
import time

import bpy

from blender_mcp.bundled import addon


def start_server(port):
    server = addon.BlenderMCPServer(port=port)
    bpy.types.blendermcp_server = server

    # Équivalent de BlenderMCPServer.start() sans le test bpy.app.background.
    server.running = True
    server.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.socket.bind((server.host, server.port))
    server.socket.listen(5)
    server.server_thread = threading.Thread(target=server._server_loop, daemon=True)
    server.server_thread.start()
    addon._register_edit_capture_handlers()
    print(f"BlenderMCP (headless) en écoute sur {server.host}:{server.port}", flush=True)
    return server


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", type=int, default=9876)
    parser.add_argument("--blend", help="fichier .blend à ouvrir au démarrage")
    args = parser.parse_args()

    if args.blend:
        bpy.ops.wm.open_mainfile(filepath=args.blend)

    addon.register()
    server = start_server(args.port)

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    print(f"Blender {bpy.app.version_string} prêt (Ctrl+C pour arrêter)", flush=True)
    while not stop.is_set():
        server._drain_command_queue()
        time.sleep(0.05)

    server.stop()
    addon.unregister()


if __name__ == "__main__":
    main()
