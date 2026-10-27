#!/usr/bin/env python3
"""Run the RaspTank tests on Linux or Windows (with ROS 2 and the workspace loaded).

    python3 scripts/tests.py            # unit tests of every package and the app
    python3 scripts/tests.py --e2e      # and the simulated mission (~1 min)

scripts/test.sh does the same on Linux; tests_windows.bat calls this one.
"""
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = ['rasptank_core', 'rasptank_description', 'rasptank_sim', 'rasptank_perception',
            'rasptank_hw', 'rasptank_calibration']
WINDOWS = os.name == 'nt'


def pytest(folder, target, extra_env=None):
    env = dict(os.environ, PYTEST_DISABLE_PLUGIN_AUTOLOAD='1', **(extra_env or {}))
    print(f'== {folder.name}', flush=True)
    return subprocess.call([sys.executable, '-m', 'pytest', '-q', target], cwd=folder, env=env)


def start_launch(args, log):
    kwargs = {'stdout': log, 'stderr': subprocess.STDOUT}
    if WINDOWS:
        kwargs['creationflags'] = subprocess.CREATE_NEW_PROCESS_GROUP
    else:
        kwargs['start_new_session'] = True
    return subprocess.Popen(['ros2', 'launch', 'rasptank_bringup', 'sim.launch.py'] + args,
                            **kwargs)


def stop_launch(proc):
    if proc.poll() is not None:
        return
    if WINDOWS:
        proc.send_signal(signal.CTRL_BREAK_EVENT)
    else:
        os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        if WINDOWS:
            subprocess.call(['taskkill', '/F', '/T', '/PID', str(proc.pid)])
        else:
            os.killpg(proc.pid, signal.SIGKILL)


def e2e(timeout=240):
    print('== misión simulada', flush=True)
    env_domain = os.environ.setdefault('ROS_DOMAIN_ID', '77')
    log_path = ROOT / 'ros2_ws' / 'log' / 'e2e_sim.log'
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with open(log_path, 'w') as log:
        proc = start_launch(['rviz:=false', 'mission:=true', 'autostart:=true',
                             'calibration_file:=none'], log)
        try:
            time.sleep(3)
            rc = subprocess.call([sys.executable, str(ROOT / 'scripts' / 'comprobar_mision.py'),
                                  str(timeout)])
        finally:
            stop_launch(proc)
    if rc:
        print(f'registro de la simulación (dominio {env_domain}): {log_path}')
    return rc


def main():
    failed = 0
    for pkg in PACKAGES:
        failed |= pytest(ROOT / 'ros2_ws' / 'src' / pkg, 'test') != 0
    failed |= pytest(ROOT / 'tools' / 'app', 'tests', {'QT_QPA_PLATFORM': 'offscreen'}) != 0
    if '--e2e' in sys.argv:
        failed |= e2e() != 0
    print('RESULTADO:', 'FALLAN TESTS' if failed else 'todo correcto')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
