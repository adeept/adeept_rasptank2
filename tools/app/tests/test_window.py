"""The window opens and behaves without ROS (offscreen), with a fake link."""
import pytest

pytest.importorskip('PyQt5')

from PyQt5.QtWidgets import QApplication  # noqa: E402

from rasptank_app.main_window import MainWindow  # noqa: E402
from rasptank_app.status import Snapshot  # noqa: E402


class FakeLink:
    def __init__(self, driver='fake'):
        self.snap = Snapshot(driver=driver, heartbeat_at=0.0, stopped=False)
        self.sent = []

    def snapshot(self):
        import time
        self.snap.now = time.monotonic()
        self.snap.heartbeat_at = self.snap.now
        return self.snap

    def last_jpeg(self):
        return None

    def emergency_stop(self, on):
        self.sent.append(('stop', on))
        self.snap.stopped = on

    def drive(self, v, w):
        self.sent.append(('drive', v, w))

    def arm_pose(self, name):
        self.sent.append(('pose', name))

    def camera_tilt(self, r):
        pass

    def close(self):
        self.sent.append(('close',))


@pytest.fixture(scope='module')
def app():
    return QApplication.instance() or QApplication([])


def test_window_without_ros(app):
    def fail(domain):
        raise RuntimeError('no ROS here')
    w = MainWindow(fail)
    w.refresh()
    assert w.mode_label.text() == 'SIN CONEXIÓN'
    w.close()


def test_stop_and_mode(app):
    link = FakeLink()
    w = MainWindow(lambda d: link)
    w.refresh()
    assert w.mode_label.text() == 'SIMULACIÓN'
    w._drive_start(1, 0)
    w.emergency_stop()
    assert w.drive_cmd is None                    # the stop also cancels teleoperation
    assert ('stop', True) in link.sent and link.sent[-2] == ('drive', 0.0, 0.0)
    w.refresh()
    assert w.mode_label.text() == 'PARADA DE EMERGENCIA'
    w.close()
    assert link.sent[-1] == ('close',)


def test_calibration_page_without_connection(app, monkeypatch):
    from PyQt5.QtWidgets import QMessageBox
    shown = []
    monkeypatch.setattr(QMessageBox, 'warning', lambda *a: shown.append(a[2]))
    w = MainWindow(lambda d: (_ for _ in ()).throw(RuntimeError('x')))
    w.calibration.start()
    assert shown and 'ROS' in shown[0]
    w.close()
