#!/usr/bin/env python3
"""Espera a que la misión simulada termine y comprueba el resultado.

Lo usa prueba_mision_sim.sh: sale con 0 si la misión acaba en FIN con el
cubo dentro de la meta, y con 1 si falla o se acaba el tiempo.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String


def main():
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 180.0
    rclpy.init()
    node = Node('comprobar_mision')
    state = {'mission': '', 'in_goal': False}
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def on_state(msg):
        if msg.data != state['mission']:
            print(f'[misión] {msg.data}', flush=True)
        state['mission'] = msg.data

    node.create_subscription(String, 'mission/state', on_state, latched)
    node.create_subscription(Bool, 'sim/cube_in_goal',
                             lambda m: state.__setitem__('in_goal', m.data), 10)
    start = time.monotonic()
    result = 1
    while time.monotonic() - start < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        if state['mission'].startswith('FALLO'):
            print('RESULTADO: la misión ha fallado')
            break
        if state['mission'].startswith('FIN'):
            end = time.monotonic() + 1.0
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            ok = state['in_goal']
            print(f'RESULTADO: misión terminada en {time.monotonic() - start:.0f} s, cubo '
                  + ('DENTRO de la meta' if ok else 'FUERA de la meta'))
            result = 0 if ok else 1
            break
    else:
        print(f'RESULTADO: tiempo agotado ({timeout:.0f} s) en {state["mission"]!r}')
    node.destroy_node()
    rclpy.shutdown()
    return result


if __name__ == '__main__':
    sys.exit(main())
