"""Hardware drivers of the RaspTank, and fake ones for the PC.

The real drivers import the Raspberry Pi libraries (Adafruit PCA9685,
gpiozero, smbus) only when they are created, so the nodes also run on a PC
with `driver: fake` (simulation, tests).

Wiring (from the Adeept code in web/):
  PCA9685 at I2C 0x5f, 50 Hz
    channels 0-4: servos A-E (arm, hand, wrist, gripper, camera)
    channels 15/14: motor M1, 12/13: motor M2
  ultrasonic HC-SR04: trigger GPIO 23, echo GPIO 24
  line sensors: left GPIO 22, middle GPIO 27, right GPIO 17
  ADS7830 ADC at I2C 0x48, battery on channel 0 (divider 1k/(3k+1k))
"""


class FakeMotors:
    def __init__(self):
        self.throttle = (0.0, 0.0)

    def set(self, left, right):
        self.throttle = (left, right)

    def stop(self):
        self.set(0.0, 0.0)

    def close(self):
        self.stop()


class FakeServos:
    def __init__(self):
        self.degrees = {}

    def set_deg(self, channel, deg):
        self.degrees[channel] = deg

    def close(self):
        pass


def _pca9685(address, frequency):
    import busio
    from adafruit_pca9685 import PCA9685
    from board import SCL, SDA

    pca = PCA9685(busio.I2C(SCL, SDA), address=address)
    pca.frequency = frequency
    return pca


class PcaMotors:
    """Two DC motors (left and right track) on the PCA9685."""

    def __init__(self, address, frequency, left_channels, right_channels):
        from adafruit_motor import motor

        self.pca = _pca9685(address, frequency)
        self.left = motor.DCMotor(self.pca.channels[left_channels[0]],
                                  self.pca.channels[left_channels[1]])
        self.right = motor.DCMotor(self.pca.channels[right_channels[0]],
                                   self.pca.channels[right_channels[1]])
        for m in (self.left, self.right):
            m.decay_mode = motor.SLOW_DECAY

    def set(self, left, right):
        self.left.throttle = max(-1.0, min(1.0, left))
        self.right.throttle = max(-1.0, min(1.0, right))

    def stop(self):
        self.set(0.0, 0.0)

    def close(self):
        self.stop()
        self.pca.deinit()


class FakeMotorKiller:
    def __init__(self):
        self.zeroed = 0

    def zero(self):
        self.zeroed += 1


class PcaMotorKiller:
    """Switches the motor channels of the PCA9685 fully off, directly.

    Used by motor_watchdog_node when base_node stops responding. It does
    NOT re-initialise the chip (that would reset the servos too): it only
    sets the "full OFF" bit of each motor channel (LEDn_OFF_H bit 4).
    """

    LED0_OFF_H = 0x09     # + 4 * channel; bit 4 = full OFF, wins over everything else

    def __init__(self, address, channels, bus=1):
        try:
            from smbus2 import SMBus
        except ImportError:
            from smbus import SMBus
        self.bus = SMBus(bus)
        self.address = address
        self.channels = list(channels)

    def zero(self):
        # one byte per channel: works whether or not register auto-increment is on
        for ch in self.channels:
            self.bus.write_byte_data(self.address, self.LED0_OFF_H + 4 * ch, 0x10)


class PcaServos:
    """Hobby servos on the PCA9685 (500-2400 us for 0-180 degrees)."""

    def __init__(self, address, frequency, channels):
        from adafruit_motor import servo

        self.pca = _pca9685(address, frequency)
        self.servos = {ch: servo.Servo(self.pca.channels[ch], min_pulse=500, max_pulse=2400,
                                       actuation_range=180) for ch in channels}

    def set_deg(self, channel, deg):
        self.servos[channel].angle = max(0.0, min(180.0, deg))

    def close(self):
        self.pca.deinit()


class GpioUltrasonic:
    def __init__(self, trigger, echo, max_distance):
        from gpiozero import DistanceSensor

        self.sensor = DistanceSensor(echo=echo, trigger=trigger, max_distance=max_distance)

    def read(self):
        """Distance in metres (max_distance when nothing echoes)."""
        return float(self.sensor.distance)


class GpioLineSensors:
    def __init__(self, left, middle, right, active_high=True):
        from gpiozero import InputDevice

        self.pins = [InputDevice(pin=p) for p in (left, middle, right)]
        self.active_high = active_high

    def read(self):
        values = [bool(p.value) for p in self.pins]
        return values if self.active_high else [not v for v in values]


class Ads7830Battery:
    def __init__(self, address=0x48, channel=0, vref=5.2, divider=1000 / (3000 + 1000), bus=1):
        try:
            from smbus2 import SMBus
        except ImportError:
            from smbus import SMBus
        self.bus = SMBus(bus)
        self.address = address
        self.cmd = 0x84 | ((((channel << 2) | (channel >> 1)) & 0x07) << 4)
        self.vref = vref
        self.divider = divider

    def read(self):
        """Battery voltage."""
        raw = self.bus.read_byte_data(self.address, self.cmd)
        return raw / 255.0 * self.vref / self.divider


class FakeSignals:
    """LEDs and buzzer that only remember what they were told."""

    def __init__(self):
        self.colors = []
        self.tone = None

    def show(self, colors):
        self.colors = list(colors)

    def play(self, frequency):
        self.tone = frequency

    def stop(self):
        self.tone = None

    def close(self):
        self.show([(0, 0, 0)] * len(self.colors))
        self.stop()


class RealSignals:
    """WS2812 strip on SPI (MOSI = GPIO 10 for spidev0.0) and the buzzer on GPIO 18."""

    def __init__(self, spi_bus=0, spi_device=0, order='GRB', buzzer_pin=18):
        import spidev
        from gpiozero import TonalBuzzer

        self.spi = spidev.SpiDev()
        self.spi.open(spi_bus, spi_device)
        self.spi.mode = 0
        self.speed = int(8 / 1.25e-6) if spi_bus == 0 else int(8 / 1.0e-6)
        self.order = order
        self.buzzer = TonalBuzzer(buzzer_pin)
        self.count = 0

    def show(self, colors):
        from rasptank_core.signals import ws2812_spi
        self.count = len(colors)
        self.spi.xfer(ws2812_spi(colors, self.order), self.speed)

    def play(self, frequency):
        from gpiozero.tones import Tone
        self.buzzer.play(Tone(frequency=frequency))

    def stop(self):
        self.buzzer.stop()

    def close(self):
        self.show([(0, 0, 0)] * self.count)
        self.stop()
        self.spi.close()
