import adafruit_tsl2561
import busio
import board
import time
from micropython import const

def getLux():
    now = time.localtime()
    date = "%04d-%02d-%02d %02d:%02d:%02d" % (now.tm_year, now.tm_mon, now.tm_mday, now.tm_hour, now.tm_min, now.tm_sec)
    lux_data = []
    print(date)
    print()
    for i in tsl:    
        print("Chip ID = {}".format(i.chip_id))

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
        lux_data.append(str(lux))
        # Print results


        if lux is not None:
            print("Lux = {}".format(lux))
        else:
            print("Lux value is None. Possible sensor underrange or overrange.")

        # Disble the light sensor (to save power)
        i.enabled = False
    lux_data = date + "|" + "|".join(lux_data)
    print(lux_data)
    # client.publish("Database/bright/save", lux_data)
    
# ---Lightsensor---
# Create the I2C bus
i2c = busio.I2C(board.SCL, board.SDA)
print("i2c_scan", i2c.scan())
print("scl, sda", board.SCL, board.SDA)

# Create the TSL2561 instance, passing in the I2C bus
tsl = [adafruit_tsl2561.TSL2561(i2c, address=const(0x29)), adafruit_tsl2561.TSL2561(i2c)]
print(f'tsl test information in data => {tsl}')
while True:
    getLux()
