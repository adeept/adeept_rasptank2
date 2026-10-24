#!/usr/bin/env python3
"""Genera el marcador ArUco de la meta listo para imprimir a tamaño real.

    python3 scripts/generar_marcador.py                 # id 0, 8 cm -> marcador_0.png
    python3 scripts/generar_marcador.py --id 3 --lado 10

Imprime el PNG al 100 % (sin "ajustar a la página") y mide el cuadrado
negro: debe medir --lado cm, igual que marker_size en perception.yaml.
Pégalo en un cartón rígido de unos 14 x 16 cm, de pie sobre el suelo.
"""
import argparse
import struct
import zlib

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--id', type=int, default=0)
    parser.add_argument('--lado', type=float, default=8.0, help='cm del cuadrado negro')
    parser.add_argument('--diccionario', default='DICT_4X4_50')
    parser.add_argument('--dpi', type=int, default=300)
    args = parser.parse_args()

    ident = getattr(cv2.aruco, args.diccionario)
    if hasattr(cv2.aruco, 'getPredefinedDictionary'):
        dictionary = cv2.aruco.getPredefinedDictionary(ident)
    else:
        dictionary = cv2.aruco.Dictionary_get(ident)
    px = int(round(args.lado / 2.54 * args.dpi))
    if hasattr(cv2.aruco, 'generateImageMarker'):
        marker = cv2.aruco.generateImageMarker(dictionary, args.id, px)
    else:
        marker = cv2.aruco.drawMarker(dictionary, args.id, px)
    margin = px // 4                      # borde blanco: el detector lo necesita
    page = np.full((px + 2 * margin + 60, px + 2 * margin), 255, np.uint8)
    page[margin:margin + px, margin:margin + px] = marker
    cv2.putText(page, f'{args.diccionario} id {args.id} - {args.lado:g} cm', (margin, px + margin + 45),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, 0, 2, cv2.LINE_AA)
    name = f'marcador_{args.id}.png'
    # guarda los dpi para que se imprima al tamaño correcto
    _, png = cv2.imencode('.png', page)
    data = bytearray(png.tobytes())
    ppm = int(round(args.dpi / 0.0254))
    phys = b'pHYs' + struct.pack('>IIB', ppm, ppm, 1)
    chunk = struct.pack('>I', 9) + phys + struct.pack('>I', zlib.crc32(phys) & 0xffffffff)
    data[33:33] = chunk                   # justo después de la cabecera IHDR
    with open(name, 'wb') as f:
        f.write(data)
    print(f'{name}: {page.shape[1]}x{page.shape[0]} px a {args.dpi} ppp')


if __name__ == '__main__':
    main()
