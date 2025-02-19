import time
import board
import adafruit_ads7830.ads7830 as ADC
from adafruit_ads7830.analog_in import AnalogIn

i2c = board.I2C()

# Initialize ADS7830
adc = ADC.ADS7830(i2c,0x48)  #default is 0x48
chan1 = AnalogIn(adc, 1)
chan2 = AnalogIn(adc, 2)

chan3 = AnalogIn(adc, 3)
chan4 = AnalogIn(adc, 4)
chan5 = AnalogIn(adc, 5)
chan6 = AnalogIn(adc, 6)
chan7 = AnalogIn(adc, 7)
chan0 = AnalogIn(adc, 0)

if __name__ == "__main__":

    while True:
        print(f"battery level = {chan0.value/65535*8.4:.2f} V")

        time.sleep(0.5)

