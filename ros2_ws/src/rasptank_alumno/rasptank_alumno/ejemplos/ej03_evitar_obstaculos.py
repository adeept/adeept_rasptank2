"""Ejemplo 3: avanzar y esquivar obstáculos con el ultrasonido.

    ros2 launch rasptank_bringup sim.launch.py world:=vacio    (o mision)
    ros2 run rasptank_alumno ej03_evitar_obstaculos

Ctrl+C para terminar. Prueba: gira hacia el lado con más espacio
(mira a izquierda y derecha antes de decidir).
"""
import random

from rasptank_alumno.robot import Robot

DISTANCIA_SEGURA = 0.30   # metros


def main():
    robot = Robot()
    try:
        while True:
            d = robot.distancia()
            if d < DISTANCIA_SEGURA:
                print(f'Obstáculo a {d:.2f} m: giro')
                robot.parar()
                robot.avanzar(-0.05)
                robot.girar(random.choice([-1, 1]) * random.uniform(60, 120))
            else:
                robot.mover(0.12)
            robot.esperar(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        robot.cerrar()


if __name__ == '__main__':
    main()
