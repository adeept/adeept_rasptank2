import pytest

from rasptank_core.signals import note_frequency, parse_tune, ws2812_spi


def test_ws2812_encoding():
    data = ws2812_spi([(255, 0, 1)], order='GRB')
    assert len(data) == 24
    assert data[:8] == [0x80] * 8                 # G = 0
    assert data[8:16] == [0xF8] * 8               # R = 255
    assert data[16:24] == [0x80] * 7 + [0xF8]     # B = 1
    assert ws2812_spi([(255, 255, 255)], brightness=0.0) == [0x80] * 24
    with pytest.raises(ValueError):
        ws2812_spi([(1, 2, 3)], order='XYZ')


def test_notes_and_tunes():
    assert note_frequency('A4') == pytest.approx(440.0)
    assert note_frequency('A5') == pytest.approx(880.0)
    assert note_frequency('C4') == pytest.approx(261.63, abs=0.01)
    assert note_frequency('C#4') == pytest.approx(note_frequency('Db4'))
    tune = parse_tune('C5:0.2 -:0.1 E5')
    assert tune[1] == (None, 0.1) and tune[2][1] == 0.25
    with pytest.raises(ValueError):
        parse_tune('H4')
    with pytest.raises(ValueError):
        parse_tune('C4:99')
