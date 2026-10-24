"""The GUI calibration flow, manual and automatic, on the fake robot."""
import math

import pytest

from rasptank_calibration.flow import Flow
from rasptank_core.kinematics import Pose2D

from test_procedure import FakeSession


class TapeSession(FakeSession):
    """FakeSession with the virtual tape measure of the simulation."""

    simulated = True

    def truth_mark(self):
        p = self.truth
        return (p.x, p.y, p.yaw)

    def truth_since(self, start):
        p = self.truth
        dx, dy = p.x - start[0], p.y - start[1]
        c, s = math.cos(-start[2]), math.sin(-start[2])
        return (100 * (c * dx - s * dy), 100 * (s * dx + c * dy), math.degrees(p.yaw - start[2]))

    def camera_tilt(self, radians):
        self.tilt = radians

    def save(self):
        self.saved = self.cal
        return 'guardado'


def test_manual_flow_with_virtual_tape():
    s = TapeSession()
    flow = Flow(s, auto=False, rounds=2, virtual_tape=True)
    flow.start()
    kinds = ['dead_band', 'straight', 'rotation', 'straight', 'rotation', 'verify']
    assert [st.kind for st in flow.steps] == kinds
    while not flow.done:
        out = flow.run()
        if out.needs_input:
            flow.submit({f.key: f.sim_value for f in flow.step.fields})
        if flow.done or flow.step.kind == 'rotation':
            s.truth = Pose2D()              # put it back on the mark between tests
    assert flow.steps[-1].report.startswith('cuadrado')
    error = float(flow.steps[-1].report.split('termina a ')[1].split(' cm')[0])
    assert error < 5.0
    assert flow.save() == 'guardado'


def test_auto_flow_and_discard():
    s = TapeSession()
    flow = Flow(s, auto=True, rounds=2, battery=True)
    flow.start()
    assert s.tilt == 0.0
    while not flow.done:
        assert not flow.run().needs_input
    assert all(st.status == 'hecho' for st in flow.steps)
    assert s.cal.battery_reference == pytest.approx(8.1)
    flow.discard()
    assert s.cal == flow.original


def test_submit_needs_a_movement_first_and_skip():
    flow = Flow(TapeSession(), auto=False, dead_band=False)
    flow.start()
    with pytest.raises(RuntimeError):
        flow.submit({'measured': 100, 'lateral': 0})
    flow.skip()
    assert flow.steps[0].status == 'saltado' and flow.step.kind == 'rotation'
