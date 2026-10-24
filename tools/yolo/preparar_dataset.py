#!/usr/bin/env python3
"""Junta datasets YOLO (fotos reales etiquetadas, imágenes de la simulación)
en uno solo, con su reparto entrenamiento/validación y su data.yaml.

    python3 preparar_dataset.py --salida ~/dataset \\
        --fuente ~/etiquetadas_roboflow --fuente ~/dataset_sim:0.5

Cada --fuente es una carpeta con imágenes y etiquetas YOLO (con subcarpetas
images/ y labels/, en cualquier reparto: train, val, valid, test o ninguno).
Detrás de ":" va la fracción de esa fuente que se usa (0.5 = la mitad, al
azar): así se prueba cuánto ayudan las imágenes sintéticas.

La validación sale SOLO de la primera fuente (las fotos reales): lo que
importa es cómo funciona con la cámara de verdad.
"""
import argparse
import random
import shutil
from pathlib import Path

IMAGE_EXT = {'.jpg', '.jpeg', '.png', '.bmp'}


def pairs(folder):
    """(imagen, etiqueta o None) de una carpeta YOLO."""
    folder = Path(folder).expanduser()
    out = []
    for image in sorted(folder.rglob('*')):
        if image.suffix.lower() not in IMAGE_EXT:
            continue
        parts = list(image.parts)
        if 'images' in parts:
            i = len(parts) - 1 - parts[::-1].index('images')
            label = Path(*parts[:i], 'labels', *parts[i + 1:]).with_suffix('.txt')
        else:
            label = image.with_suffix('.txt')
        out.append((image, label if label.exists() else None))
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--salida', required=True)
    parser.add_argument('--fuente', action='append', required=True,
                        help='carpeta[:fracción]; la primera es la de validación')
    parser.add_argument('--val', type=float, default=0.2, help='fracción de la primera fuente')
    parser.add_argument('--clases', default='cubo,meta')
    parser.add_argument('--semilla', type=int, default=1)
    args = parser.parse_args(argv)

    rng = random.Random(args.semilla)
    out = Path(args.salida).expanduser()
    if out.exists():
        shutil.rmtree(out)
    for split in ('train', 'val'):
        (out / 'images' / split).mkdir(parents=True)
        (out / 'labels' / split).mkdir(parents=True)
    counts = {'train': 0, 'val': 0}
    for n, source in enumerate(args.fuente):
        path, _, fraction = source.partition(':')
        items = pairs(path)
        rng.shuffle(items)
        if fraction:
            items = items[:int(len(items) * float(fraction))]
        n_val = int(len(items) * args.val) if n == 0 else 0
        for k, (image, label) in enumerate(items):
            split = 'val' if k < n_val else 'train'
            name = f'f{n}_{k:05d}'
            shutil.copy(image, out / 'images' / split / (name + image.suffix.lower()))
            target = out / 'labels' / split / (name + '.txt')
            if label is not None:
                shutil.copy(label, target)
            else:
                target.write_text('')            # an image without objects
            counts[split] += 1
        print(f'{path}: {len(items)} imágenes ({n_val} a validación)')
    names = [c.strip() for c in args.clases.split(',') if c.strip()]
    (out / 'data.yaml').write_text(
        f'path: {out.resolve()}\ntrain: images/train\nval: images/val\nnames:\n'
        + ''.join(f'  {i}: {name}\n' for i, name in enumerate(names)))
    print(f'{out}: {counts["train"]} de entrenamiento, {counts["val"]} de validación')
    if counts['val'] == 0:
        print('AVISO: no hay validación; pon primero la fuente con fotos reales')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
