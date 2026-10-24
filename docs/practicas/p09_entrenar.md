# Práctica 9 — Entrenar un detector

**Objetivo:** recorrer el proceso de aprendizaje automático: datos,
etiquetas, entrenamiento, evaluación y despliegue. Guía completa:
[tools/yolo/README.md](../../tools/yolo/README.md).

## 1. Datos

- **Sintéticos** (todo el grupo): `ros2 run rasptank_sim dataset_sim --out
  ~/dataset_sim --images 600`.
- **Reales** (por parejas, con el robot): `capture_node` mientras lo movéis
  alrededor del cubo. 50 fotos cada pareja, con luces y fondos distintos y
  otros objetos rojos.

## 2. Etiquetar

Roboflow, Label Studio o CVAT: un rectángulo por objeto, clases `cubo` y `meta`.

## 3. Entrenar (Google Colab o un PC con GPU)

```bash
python3 tools/yolo/preparar_dataset.py --salida ~/dataset \
    --fuente ~/fotos_etiquetadas --fuente ~/dataset_sim:0.5
python3 tools/yolo/entrenar_yolo.py --data ~/dataset/data.yaml --epochs 60
```

## 4. Experimentos (un grupo cada uno)

| Grupo | Entrenamiento | Validación |
|---|---|---|
| A | solo sintéticas | fotos reales |
| B | solo fotos reales | fotos reales |
| C | reales + sintéticas | fotos reales |
| D | como C, `--desde-cero` | fotos reales |

## 5. Desplegar

```bash
./scripts/sim.sh mision cube_detector:=yolo yolo_model:=$HOME/cubo_meta.onnx
```

y con el robot real, `pc.launch.py` con los mismos argumentos.

## Preguntas

1. ¿Qué grupo saca mejor mAP50? ¿Y mejor resultado en el robot real? ¿Coinciden?
2. ¿Por qué el modelo de solo sintéticas falla con la cámara real?
3. ¿Qué es el mAP? ¿Y un falso positivo? Pon un ejemplo de cada en vuestras pruebas.
4. ¿Compensa frente al detector por color? ¿Cuándo sí y cuándo no?

## Entrega

La tabla de resultados de todos los grupos, la matriz de confusión del
vuestro y las respuestas.
