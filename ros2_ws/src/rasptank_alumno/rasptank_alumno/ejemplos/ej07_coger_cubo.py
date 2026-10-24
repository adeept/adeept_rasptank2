"""Ejemplo 7: buscar el cubo con la cámara, acercarse y cogerlo.

    ros2 launch rasptank_bringup sim.launch.py
    ros2 run rasptank_alumno ej07_coger_cubo

Es una versión sencilla de la misión (rasptank_core/mission.py). Retos:
  - llevarlo después a la meta: robot.ver('meta')
  - comprobar si lo has cogido de verdad (¿sigue viéndose en el suelo?)
"""
from rasptank_alumno.robot import ALCANCE_PINZA, Robot

AVANCE_FINAL = 0.12       # m: desde aquí el cubo sale de la imagen


def buscar(robot, objeto):
    """Gira a saltos hasta ver el objeto."""
    for _ in range(40):
        if robot.ver(objeto):
            return True
        robot.girar(15)
        robot.esperar(0.6)          # el detector necesita una imagen quieta
    return False


def main():
    robot = Robot()
    robot.brazo('buscar')
    if not buscar(robot, 'cubo'):
        print('No encuentro el cubo')
        robot.cerrar()
        return
    while True:
        cubo = robot.ver('cubo')
        if cubo is None:
            robot.parar()
            if not buscar(robot, 'cubo'):
                break
            continue
        print(f"cubo a {cubo['distancia']:.2f} m, {cubo['angulo']:.0f}°")
        if cubo['adelante'] < ALCANCE_PINZA + AVANCE_FINAL and abs(cubo['angulo']) < 3:
            break
        giro = 2.0 * cubo['angulo']                     # control proporcional
        cerca = cubo['adelante'] < ALCANCE_PINZA + AVANCE_FINAL
        velocidad = 0.08 if abs(cubo['angulo']) < 10 and not cerca else 0.0
        robot.mover(velocidad, giro)
        robot.esperar(0.05)
    robot.parar()
    # el cubo ya no se verá: último tramo a ciegas hasta tenerlo bajo la pinza
    robot.avanzar(cubo['adelante'] - ALCANCE_PINZA, 0.04)
    robot.brazo('pre_coger')
    robot.cerrar_pinza()
    robot.brazo('transportar')
    print('¡Cubo cogido!')
    robot.cerrar()


if __name__ == '__main__':
    main()
