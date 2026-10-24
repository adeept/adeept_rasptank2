#!/usr/bin/env python3
"""Entrena un detector YOLO (cubo y meta) y lo exporta a ONNX para el PC.

    pip install ultralytics            # en el PC con GPU, o en Google Colab
    python3 entrenar_yolo.py --data ~/dataset/data.yaml --epochs 60

Al terminar deja `cubo_meta.onnx`, que usa el detector:

    ros2 launch rasptank_bringup pc.launch.py ...  con perception.yaml:
        cube_detector: yolo
        yolo_model: /ruta/a/cubo_meta.onnx

Por defecto parte de los pesos preentrenados `yolov8n.pt` (se descargan solos;
con --desde-cero entrena sin ellos, peor pero sin internet).
Las imágenes son de 640x480; se entrena a --imgsz 320 para que vaya rápido.
"""
import argparse
import os
import shutil


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--data', required=True, help='data.yaml del dataset (formato YOLO)')
    parser.add_argument('--epochs', type=int, default=60)
    parser.add_argument('--imgsz', type=int, default=320)
    parser.add_argument('--batch', type=int, default=16)
    parser.add_argument('--modelo', default='yolov8n', help='yolov8n, yolo11n, ...')
    parser.add_argument('--desde-cero', action='store_true', help='sin pesos preentrenados')
    parser.add_argument('--salida', default='cubo_meta.onnx')
    parser.add_argument('--device', default=None, help='0 para la GPU, cpu para la CPU')
    args = parser.parse_args()

    from ultralytics import YOLO

    model = YOLO(args.modelo + ('.yaml' if args.desde_cero else '.pt'))
    model.train(data=os.path.expanduser(args.data), epochs=args.epochs, imgsz=args.imgsz,
                batch=args.batch, device=args.device, plots=False, name='rasptank')
    metrics = model.val(data=os.path.expanduser(args.data), imgsz=args.imgsz, plots=False)
    print(f'mAP50: {metrics.box.map50:.3f}  mAP50-95: {metrics.box.map:.3f}')
    onnx = model.export(format='onnx', imgsz=args.imgsz, opset=12, simplify=True)
    shutil.copy(onnx, args.salida)
    print(f'modelo exportado: {os.path.abspath(args.salida)}')


if __name__ == '__main__':
    main()
