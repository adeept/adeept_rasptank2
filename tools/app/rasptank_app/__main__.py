"""RaspTank app:  ./scripts/app.sh   (or python3 -m rasptank_app from tools/app).

    --dominio N   ROS domain to connect to (41 simulation by default)
    --smoke S     open, refresh for S seconds and close (tests without screen)
"""
import argparse
import sys

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication

from .main_window import MainWindow
from .status import SIM_DOMAIN


def connect(domain):
    from .ros_link import RosLink
    return RosLink(domain)


def main():
    parser = argparse.ArgumentParser(description='Aplicación del RaspTank')
    parser.add_argument('--dominio', type=int, default=SIM_DOMAIN)
    parser.add_argument('--smoke', type=float, default=0.0)
    parser.add_argument('--captura', default='', help='guarda una imagen de la ventana (con --smoke)')
    args, qt_args = parser.parse_known_args()
    app = QApplication([sys.argv[0]] + qt_args)
    app.setApplicationName('RaspTank')
    app.setStyle('Fusion')
    window = MainWindow(connect, args.dominio)
    window.show()
    if args.smoke:
        def finish():
            if args.captura:
                window.grab().save(args.captura)
            window.close()
        QTimer.singleShot(int(args.smoke * 1000), finish)
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
