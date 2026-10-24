"""Ejemplo 4: seguir una línea negra con los tres sensores infrarrojos.

    ros2 launch rasptank_bringup sim.launch.py world:=linea
    ros2 run rasptank_alumno ej04_seguir_linea

Control "todo o nada": si la línea se va a un lado, gira hacia ese lado.
Prueba: ajusta VELOCIDAD y GIRO, o haz un control proporcional.
"""
from rasptank_alumno.robot import Robot

VELOCIDAD = 0.08   # m/s
GIRO = 60.0        # grados/s


def main():
    robot = Robot()
    ultimo_giro = GIRO
    try:
        while True:
            izquierda, centro, derecha = robot.linea()
            if centro and not izquierda and not derecha:
                robot.mover(VELOCIDAD)                     # recto
            elif izquierda:
                robot.mover(VELOCIDAD / 2, GIRO)           # línea a la izquierda
                ultimo_giro = GIRO
            elif derecha:
                robot.mover(VELOCIDAD / 2, -GIRO)          # línea a la derecha
                ultimo_giro = -GIRO
            elif centro:
                robot.mover(VELOCIDAD)
            else:
                robot.mover(0.0, ultimo_giro)              # perdida: busca girando
            robot.esperar(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        robot.cerrar()


if __name__ == '__main__':
    main()
