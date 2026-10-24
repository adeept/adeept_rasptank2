"""Ejemplo 6: mover el brazo, la pinza y la cámara.

    ros2 run rasptank_alumno ej06_brazo

Las posturas con nombre están en rasptank_bringup/config/robot.yaml.
"""
from rasptank_alumno.robot import Robot


def main():
    robot = Robot()
    robot.brazo('buscar')
    robot.brazo('pre_coger')
    robot.cerrar_pinza()
    robot.brazo('transportar')
    robot.abrir_pinza()
    robot.articulaciones({'wrist_roll_joint': 45})      # gira la pinza 45°
    robot.articulaciones({'wrist_roll_joint': 0})
    robot.camara(0)       # cámara al frente
    robot.camara(20)      # hacia el suelo
    robot.brazo('plegado')
    robot.cerrar()


if __name__ == '__main__':
    main()
