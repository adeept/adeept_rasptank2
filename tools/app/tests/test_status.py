from rasptank_app.status import Snapshot, base_alive, diagnosis, fmt, mode


def test_modes():
    assert mode(Snapshot(now=10.0))[0] == 'SIN CONEXIÓN'
    alive = dict(now=10.0, heartbeat_at=9.8)
    assert mode(Snapshot(driver='fake', **alive))[0] == 'SIMULACIÓN'
    assert mode(Snapshot(driver='pca9685', **alive))[0] == 'ROBOT REAL'
    assert mode(Snapshot(driver='pca9685', stopped=True, **alive))[0] == 'PARADA DE EMERGENCIA'
    assert not base_alive(Snapshot(now=10.0, heartbeat_at=5.0))       # stale heartbeat


def test_diagnosis_and_format():
    assert 'latido' in diagnosis(Snapshot(now=1.0))
    ok = Snapshot(now=1.0, heartbeat_at=1.0, camera_hz=10.0, distance=1.2)
    assert diagnosis(ok) == 'Todo responde.'
    assert 'cámara' in diagnosis(Snapshot(now=1.0, heartbeat_at=1.0))
    assert fmt(None, '{:.1f}') == '—' and fmt(1.25, '{:.1f}') == '1.2'
