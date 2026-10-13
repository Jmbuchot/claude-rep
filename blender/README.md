# Blender + MCP communautaire

Pilotage de Blender par Claude via le serveur MCP communautaire
[ahujasid/mcp-for-blender](https://github.com/ahujasid/mcp-for-blender)
(anciennement `blender-mcp`).

Le serveur MCP (`uvx mcp-for-blender`) parle à un add-on qui tourne dans
Blender. Les deux communiquent par un socket TCP sur `localhost:9876`.

```
Claude ──stdio──> mcp-for-blender ──TCP 9876──> add-on ──> bpy (Blender)
```

## Dans un conteneur sans écran (session cloud Claude Code)

Blender est installé comme module Python (`bpy` 5.2.2 LTS, depuis PyPI).
L'add-on exige normalement l'interface graphique : `headless_bridge.py` le
charge, ouvre le socket et exécute les commandes sur le thread principal.

```bash
./blender/install.sh --start     # installe bpy + mcp-for-blender, lance le pont
```

`.mcp.json` à la racine déclare le serveur `blender` pour Claude Code. Il est
pris en compte au démarrage de la session suivante.

Ce qui marche sans écran : `get_scene_info`, `execute_blender_code` (tout
`bpy`, y compris le rendu Cycles CPU vers un fichier PNG), `search_assets` et
`import_asset`. Ce qui ne marche pas : `look`, parce que la capture du viewport
demande une fenêtre. Pour voir la scène, on fait un rendu avec
`execute_blender_code`.

## Blender graphique dans le conteneur

`./blender/launch_gui.sh` installe Blender 4.0.2 (paquet Ubuntu) et le lance
sur un écran virtuel Xvfb (`:99`) avec l'add-on activé. Arrêter d'abord le pont
headless, qui occupe le même port. Dans ce mode, l'outil `look` fonctionne.

## Sur un poste de travail (Windows, macOS, Linux)

1. Installer Blender 3.0 ou plus récent depuis <https://www.blender.org/download/>.
2. Installer [uv](https://docs.astral.sh/uv/).
3. Récupérer `addon.py` :
   <https://github.com/ahujasid/mcp-for-blender/blob/main/addon.py>
4. Dans Blender, ouvrir Edit > Preferences > Add-ons > Install from Disk,
   choisir `addon.py` et cocher « MCP for Blender ».
5. Dans la vue 3D, touche N, onglet « MCP for Blender », cliquer sur
   « Connect to Claude ».
6. Déclarer le serveur MCP :
   - Claude Code : `claude mcp add blender -- uvx mcp-for-blender`
   - Claude Desktop : ajouter le bloc `mcpServers` de `.mcp.json` à
     `claude_desktop_config.json`.

## Télémétrie

`mcp-for-blender` envoie de la télémétrie anonyme par défaut. Ici, elle est
coupée avec `DISABLE_TELEMETRY=true`.
