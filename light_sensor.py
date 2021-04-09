# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
import busio
import adafruit_tsl2561
import RPi.GPIO as GPIO
from micropython import const
import datetime

now = datetime.datetime.now()
print(now)

# Create the I2C bus
i2c = busio.I2C(board.SCL, board.SDA)
print(i2c.scan())
print(board.SCL, board.SDA)



# Create the TSL2561 instance, passing in the I2C bus
tsl = [adafruit_tsl2561.TSL2561(i2c), adafruit_tsl2561.TSL2561(i2c, address=const(0x29))]
# Print chip info
print(tsl)

for i in tsl:
    
    print("Chip ID = {}".format(i.chip_id))
    print("Enabled = {}".format(i.enabled))
    print("Gain = {}".format(i.gain))
    print("Integration time = {}".format(i.integration_time))


    print("Configuring TSL2561...")
    # Enable the light sensor
    i.enabled = True
    time.sleep(1)

    # Set gain 0=1x, 1=16x
    i.gain = 0

    # Set integration time (0=13.7ms, 1=101ms, 2=402ms, or 3=manual)
    i.integration_time = 1

    print("Getting readings...")

    # Get raw (luminosity) readings individually
    broadband = i.broadband
    infrared = i.infrared

    # Get raw (luminosity) readings using tuple unpacking
    # broadband, infrared = tsl.luminosity

    # Get computed lux value (tsl.lux can return None or a float)
    lux = i.lux

    # Print results
    print("Enabled = {}".format(i.enabled))
    print("Gain = {}".format(i.gain))
    print("Integration time = {}".format(i.integration_time))
    print("Broadband = {}".format(broadband))
    print("Infrared = {}".format(infrared))

    if lux is not None:
        print("Lux = {}".format(lux))
    else:
        print("Lux value is None. Possible sensor underrange or overrange.")

    # Disble the light sensor (to save power)
    i.enabled = False
