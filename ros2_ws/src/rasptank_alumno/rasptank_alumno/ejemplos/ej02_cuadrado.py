"""Ejemplo 2: dibujar un cuadrado con un bucle.

    ros2 run rasptank_alumno ej02_cuadrado

Prueba: cambia el lado, haz un triángulo (¿cuánto hay que girar?) o un
polígono de N lados. En el robot real, ¿vuelve al mismo sitio? ¿Por qué no?
"""
from rasptank_alumno.robot import Robot

LADO = 0.4


def main():
    robot = Robot()
    for lado in range(4):
        print(f'Lado {lado + 1}')
        robot.avanzar(LADO)
        robot.girar(90)
    x, y, angulo = robot.posicion()
    print(f'Error al volver: x={x:.3f} m, y={y:.3f} m, ángulo={angulo:.1f}°')
    robot.cerrar()


if __name__ == '__main__':
    main()
