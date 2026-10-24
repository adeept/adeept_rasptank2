"""Ejemplo 8: luces y sonido como semáforo de distancia.

    ros2 run rasptank_alumno ej08_luces_sonido

Verde si no hay nada cerca, amarillo a menos de 50 cm, rojo y pitido a menos
de 25 cm. Mueve el robot con la app o con el teclado para probarlo.
Ctrl+C para terminar. En la simulación, la luz se ve en RViz sobre el robot.
"""
import math

from rasptank_alumno.robot import Robot


def main():
    robot = Robot()
    robot.melodia('C5:0.15 E5:0.15 G5:0.3')       # saludo
    anterior = None
    try:
        while True:
            d = robot.distancia()
            if math.isnan(d):              # todavía no ha llegado ninguna medida
                robot.esperar(0.1)
                continue
            if d < 0.25:
                color = 'rojo'
            elif d < 0.50:
                color = 'amarillo'
            else:
                color = 'verde'
            if color != anterior:
                print(f'{d:.2f} m: {color}')
                if color == 'rojo':
                    robot.luces(255, 0, 0)
                    robot.pitar('A5', 0.3)
                elif color == 'amarillo':
                    robot.luces(255, 160, 0)
                else:
                    robot.luces(0, 255, 0)
                anterior = color
            robot.esperar(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        robot.apagar_luces()
        robot.cerrar()


if __name__ == '__main__':
    main()
