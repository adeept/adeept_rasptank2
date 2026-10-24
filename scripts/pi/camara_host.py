#!/usr/bin/env python3
"""Envía la cámara de la Raspberry Pi al camera_node que corre en Docker.

Se ejecuta en Raspberry Pi OS (fuera de Docker), donde picamera2 funciona:

    python3 scripts/pi/camara_host.py            # escucha en 127.0.0.1:8555

y en el contenedor: camera_node con backend tcp y source 127.0.0.1:8555
(robot.launch.py camera_backend:=tcp camera_source:=127.0.0.1:8555).

Protocolo: cada imagen es un entero de 4 bytes (big endian) con el tamaño y
después el JPEG. Solo atiende a un cliente a la vez.
"""
import argparse
import socket
import struct
import time

import cv2
from picamera2 import Picamera2


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8555)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=float, default=10.0)
    parser.add_argument('--quality', type=int, default=70)
    args = parser.parse_args()

    cam = Picamera2()
    cam.configure(cam.create_video_configuration(
        main={'size': (args.width, args.height), 'format': 'RGB888'}))
    cam.start()
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.host, args.port))
    server.listen(1)
    print(f'cámara en {args.host}:{args.port}')
    period = 1.0 / args.fps
    try:
        while True:
            conn, addr = server.accept()
            print('cliente', addr)
            try:
                while True:
                    t0 = time.monotonic()
                    frame = cam.capture_array()          # 'RGB888' es BGR en memoria
                    ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, args.quality])
                    if ok:
                        data = jpeg.tobytes()
                        conn.sendall(struct.pack('>I', len(data)) + data)
                    time.sleep(max(0.0, period - (time.monotonic() - t0)))
            except (BrokenPipeError, ConnectionResetError):
                print('cliente desconectado')
            finally:
                conn.close()
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()


if __name__ == '__main__':
    main()
