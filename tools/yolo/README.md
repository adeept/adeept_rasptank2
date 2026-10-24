# Entrenar un detector YOLO para el cubo

El detector por color funciona sin entrenar, pero falla con otra luz u otros
objetos rojos. Con un modelo propio los alumnos recorren todo el proceso de
aprendizaje automático: datos → etiquetas → entrenamiento → evaluación →
despliegue en el robot.

## 1. Capturar imágenes

Con el robot (o la simulación) en marcha:

```bash
ros2 run rasptank_perception capture_node --ros-args -p out_dir:=~/dataset_real -p period:=1.0
ros2 run teleop_twist_keyboard teleop_twist_keyboard     # mueve el robot alrededor del cubo
```

Consejos: 200–500 imágenes; el cubo a distintas distancias (0,2–1,5 m),
ángulos y posiciones en la imagen; distintas luces y fondos; algunas sin
cubo; otros objetos rojos para que aprenda a no confundirlos.

## 2. Etiquetar

Con [Roboflow](https://roboflow.com), [Label Studio](https://labelstud.io) o
[CVAT](https://www.cvat.ai): un rectángulo por objeto, clases en este orden:

```
0 cubo
1 meta
```

Exporta en formato **YOLO** (carpetas `images/` y `labels/` y un
`data.yaml`), con un 80 % para entrenar y un 20 % para validar.

**Dataset sintético:** el simulador genera imágenes ya etiquetadas:

```bash
ros2 run rasptank_sim dataset_sim --out ~/dataset_sim --images 600
```

Sirve para empezar y para comparar: un modelo entrenado solo con imágenes
sintéticas funciona mal con la cámara real (*domain gap*). Mezclarlas con
fotos reales sí ayuda.

## 3. Entrenar

En un PC con GPU, o en Google Colab (gratis, con GPU):

```bash
pip install ultralytics
python3 tools/yolo/entrenar_yolo.py --data ~/dataset_real/data.yaml --epochs 60
```

Parte de los pesos preentrenados `yolov8n.pt` (aprendizaje por
transferencia): con pocos cientos de imágenes basta. Con GPU tarda 10–20
minutos; en CPU, mucho más. Al final escribe las métricas (mAP50) y deja
`cubo_meta.onnx`.

## 4. Evaluar

Mira en `runs/detect/rasptank*/`: las curvas de pérdida, la matriz de
confusión y las predicciones sobre la validación. Preguntas para clase:
¿qué imágenes falla? ¿falsos positivos con otros objetos rojos? ¿mejora con
más datos o con más épocas? ¿y si quitas el preentrenamiento
(`--desde-cero`)?

## 5. Usarlo en el robot

Al lanzar:

```bash
ros2 launch rasptank_bringup pc.launch.py cube_detector:=yolo yolo_model:=$HOME/cubo_meta.onnx
./scripts/sim.sh mision cube_detector:=yolo yolo_model:=$HOME/cubo_meta.onnx   # en simulación
```

(la confianza mínima, `yolo_confidence`, está en `config/perception.yaml`).
El detector usa OpenCV DNN: el PC no necesita PyTorch. El registro
del detector dice cuántos milisegundos tarda por imagen. La meta se sigue
detectando con el marcador ArUco, que además da la distancia exacta.
