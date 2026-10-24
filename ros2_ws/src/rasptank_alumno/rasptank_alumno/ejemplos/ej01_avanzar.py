"""Ejemplo 1: avanzar, girar y volver.

    ros2 run rasptank_alumno ej01_avanzar
"""
from rasptank_alumno.robot import Robot


def main():
    robot = Robot()
    print('Empiezo en', robot.posicion())
    robot.avanzar(0.5)          # medio metro adelante
    robot.girar(180)            # media vuelta
    robot.avanzar(0.5)          # y vuelta
    print('Termino en', robot.posicion())
    robot.cerrar()


if __name__ == '__main__':
    main()
