"""Ejemplo 5: recorrer una ruta de puntos con la odometría.

    ros2 launch rasptank_bringup sim.launch.py world:=vacio
    ros2 run rasptank_alumno ej05_ruta

Los puntos están en metros en el mapa de la odometría (el robot empieza
en (0, 0) mirando hacia +x). En RViz verás el recorrido.

Otra forma, sin programar: lanza la simulación con route:=true, haz clic
en RViz con "Publish Point" y llama a
    ros2 service call /route/start std_srvs/srv/Trigger
"""
from rasptank_alumno.robot import Robot

RUTA = [(0.5, 0.0), (0.5, 0.5), (0.0, 0.5), (0.0, 0.0)]


def main():
    robot = Robot()
    for i, (x, y) in enumerate(RUTA, start=1):
        print(f'Punto {i}: ({x}, {y})')
        robot.ir_a(x, y)
    print('Ruta terminada en', robot.posicion())
    robot.cerrar()


if __name__ == '__main__':
    main()
