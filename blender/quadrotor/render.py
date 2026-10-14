"""Rendu de l'animation (ou d'une plage d'images) depuis le .blend.

Usage :
    python render.py quadrirotor.blend images/ [--debut 1] [--fin 697]
                     [--echantillons 20] [--threads 2]

Les images déjà présentes sont sautées : on peut relancer après une
interruption, ou couper la plage entre plusieurs processus.
"""

import argparse
import os
import time

import bpy


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("blend")
    parser.add_argument("dossier")
    parser.add_argument("--debut", type=int)
    parser.add_argument("--fin", type=int)
    parser.add_argument("--echantillons", type=int)
    parser.add_argument("--threads", type=int, help="0 = tous les cœurs")
    args = parser.parse_args()

    bpy.ops.wm.open_mainfile(filepath=args.blend)
    scene = bpy.context.scene
    scene.cycles.device = "CPU"
    if args.threads:
        scene.render.threads_mode = "FIXED"
        scene.render.threads = args.threads
    if args.echantillons:
        scene.cycles.samples = args.echantillons
    first = args.debut or scene.frame_start
    last = args.fin or scene.frame_end
    os.makedirs(args.dossier, exist_ok=True)
    for f in range(first, last + 1):
        path = os.path.join(args.dossier, f"{f:04d}.png")
        if os.path.exists(path):
            continue
        t0 = time.time()
        scene.frame_set(f)
        scene.render.filepath = path
        bpy.ops.render.render(write_still=True)
        print(f"image {f}/{last} : {time.time() - t0:.1f} s", flush=True)


if __name__ == "__main__":
    main()
