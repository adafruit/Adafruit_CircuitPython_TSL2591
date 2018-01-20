# Simple demo of the TSL2591 sensor.  Will print the detected light value
# every second.
import time

import board
import busio

import adafruit_tsl2591

# Initialize the I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize the sensor.
sensor = adafruit_tsl2591.TSL2591(i2c)

# You can optionally change the gain and integration time:
#sensor.gain = adafruit_tsl2591.GAIN_LOW (1x gain)
#sensor.gain = adafruit_tsl2591.GAIN_MED (25x gain, the default)
#sensor.gain = adafruit_tsl2591.GAIN_HIGH (428x gain)
#sensor.gain = adafruit_tsl2591.GAIN_MAX (9876x gain)
#sensor.integration_time = adafruit_tsl2591.INTEGRATIONTIME_100MS (100ms, default)
#sensor.integration_time = adafruit_tsl2591.INTEGRATIONTIME_200MS (200ms)
#sensor.integration_time = adafruit_tsl2591.INTEGRATIONTIME_300MS (300ms)
#sensor.integration_time = adafruit_tsl2591.INTEGRATIONTIME_400MS (400ms)
#sensor.integration_time = adafruit_tsl2591.INTEGRATIONTIME_500MS (500ms)
#sensor.integration_time = adafruit_tsl2591.INTEGRATIONTIME_600MS (600ms)

# Read the lux and print it every second.
while True:
    lux = sensor.lux
    print('Light: {0}lux'.format(lux))
    time.sleep(1.0)
