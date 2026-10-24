"""LEDs (WS2812) and buzzer: encoding and tunes, without hardware or ROS."""
import re

ORDERS = ('RGB', 'RBG', 'GRB', 'GBR', 'BRG', 'BGR')
NOTE_RE = re.compile(r'^([A-G])([#b]?)(\d)$')
SEMITONES = {'C': -9, 'D': -7, 'E': -5, 'F': -4, 'G': -2, 'A': 0, 'B': 2}


def ws2812_spi(colors, order='GRB', brightness=1.0):
    """SPI bytes that drive a WS2812 strip (one byte per data bit).

    `colors` is a list of (r, g, b) 0-255. At 6.4 MHz each SPI byte lasts
    1.25 us, one WS2812 bit: 0b10000000 is a short pulse (0), 0b11111000 a
    long one (1). Same encoding as Adeept's Adeept_SPI_LedPixel.
    """
    if order not in ORDERS:
        raise ValueError(f'order must be one of {ORDERS}')
    out = []
    for r, g, b in colors:
        value = {'R': r, 'G': g, 'B': b}
        for channel in order:
            byte = max(0, min(255, int(round(value[channel] * brightness))))
            for bit in range(7, -1, -1):
                out.append(0xF8 if (byte >> bit) & 1 else 0x80)
    return out


def note_frequency(note):
    """Frequency (Hz) of a note such as 'A4', 'C#5' or 'Bb3'."""
    m = NOTE_RE.match(note)
    if not m:
        raise ValueError(f'nota no válida: {note!r} (ejemplos: C4, F#5, Bb3)')
    name, accidental, octave = m.groups()
    n = SEMITONES[name] + {'#': 1, 'b': -1, '': 0}[accidental] + 12 * (int(octave) - 4)
    return 440.0 * 2.0 ** (n / 12.0)


def parse_tune(text, default=0.25):
    """[(frequency or None for a rest, seconds)] from 'C5:0.2 E5 -:0.1 G5:0.4'."""
    tune = []
    for token in text.split():
        note, _, duration = token.partition(':')
        seconds = float(duration) if duration else default
        if not 0.0 < seconds <= 5.0:
            raise ValueError(f'duración fuera de rango en {token!r}')
        tune.append((None if note in ('-', 'R', 'r') else note_frequency(note), seconds))
    return tune
