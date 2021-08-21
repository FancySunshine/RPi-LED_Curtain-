import time
import RPi.GPIO as GPIO
from multiprocessing import Process, Value
import keyboard
import paho.mqtt.client as mqtt
import board  # Simple test for NeoPixels on Raspberry Pi
import neopixel
import busio
import adafruit_tsl2561
from micropython import const
from apscheduler.schedulers.background import BackgroundScheduler
from apscheduler.jobstores.base import JobLookupError
# 구현에 필요한 라이브러리 import

n = 0
rgb = [0, 230, 50] # LED 초기 색상 설정
brightness = 0.59 # LED 초기 밝기 설정

# mqtt 연결 설정
def on_connect(client, userdata, rc, properties=None):
    print("Connected with result code " + str(rc))
    client.subscribe("Curtain/ctr")
    
    client.subscribe("LED/color")
    client.subscribe("LED/bright")

# mqtt 토픽에 의한 행동 설정
def on_message(client, userdata, msg):
    global n
    global led
    
    if msg.topic == "Curtain/ctr":
        n = int(msg.payload)
    elif msg.topic == "LED/color":
        led.rgb = map(int, msg.payload.decode("utf-8").split('|'))        
        led.colorChange()
    elif msg.topic == "LED/bright":
        led.bright = float(msg.payload)
        print("asdfasfsdfBifdfasdfsdf")
        led.setBright()
    print(msg.topic + " " + str(msg.payload))


client = mqtt.Client(client_id="asdf")
client.on_connect = on_connect
client.on_message = on_message

client.connect("192.168.191.248", 1883, 60)

client.loop_start()

# 센서 사용에 필요한 GPIO 설정
GPIO.setmode(GPIO.BCM)
# Choose an open pin connected to the Data In of the NeoPixel strip, i.e. board.D18
# NeoPixels must be connected to D10, D12, D18 or D21 to work.
# DO --> 10
# LED GPIO 18번 핀으로 설정
pixel_pin = board.D18

# The number of NeoPixels
num_pixels = 24

# The order of the pixel colors - RGB or GRB. Some NeoPixels have red and green reversed!
# For RGBW NeoPixels, simply change the ORDER to RGBW or GRBW.
ORDER = neopixel.GRB

pixels = neopixel.NeoPixel(
    pixel_pin, num_pixels, brightness=brightness, auto_write=False, pixel_order=ORDER
)

class LED:
    def __init__(self, num_pixels, pixel_pin, bright, rgb):
        self.num_pixels = num_pixels
        self.pixel_pin = pixel_pin
        self.bright = bright
        self.rgb = rgb
        
    def colorChange(self):
        pixels.fill(tuple(self.rgb))
        pixels.show()           
            
    def setBright(self):
        pixels.brightness = led.bright
        pixels.show()
        print("asdfasfsdfadf")


# curtain
class step:
    def __init__(self, Pins, status, act):
        self.StepCount = 4
        self.StepCounter = 0
        self.Pins = Pins
        self.status = Value('i', status)
        self.act = Value('i', act)

    def ini(self):
        for pin in self.Pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, False)

    def up(self):
        count = int(self.status.value)

        up = [[0, 0, 0, 1], [0, 0, 1, 0], [0, 1, 0, 0], [1, 0, 0, 0]]

        try:
            if count > 0:
                for pin in range(0, 4):
                    xpin = self.Pins[pin]
                    if up[count % 4][pin] != 0:
                        GPIO.output(xpin, True)
                    else:
                        GPIO.output(xpin, False)
                count -= 1
        except KeyboardInterrupt:
            print()
        self.status.value = count
        # print(self.status.value)

    def down(self):
        count = int(self.status.value)
        down = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]

        try:
            if count < 6300:
                for pin in range(0, 4):
                    xpin = self.Pins[pin]
                    if down[count % 4][pin] != 0:
                        GPIO.output(xpin, True)
                    else:
                        GPIO.output(xpin, False)

                count += 1
        except KeyboardInterrupt:
            print()
        self.status.value = count
        # print(self.status.value)

    def ctr(self, control):
        if control == 1:
            self.down()
        else:
            self.up()


def control(steps, ctr, procs):
        if steps == 0:
            for i in range(len(ctr)):
                ctr[i] = 0
                procs[i] = Process(target=step_arr[i].ctr, args=(ctr[i],))
                procs[i].start()
        else:
            for i in range(len(ctr)):
                if i < steps:
                    ctr[i] = 1
                else:
                    ctr[i] = 0
                procs[i] = Process(target=step_arr[i].ctr, args=(ctr[i],))
                procs[i].start()
                
# get Lux from sensor                
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
    client.publish("Database/bright/save", lux_data)
        
        

# ---Lightsensor---
# Create the I2C bus
i2c = busio.I2C(board.SCL, board.SDA)
print(i2c.scan())
print(board.SCL, board.SDA)

# Create the TSL2561 instance, passing in the I2C bus
tsl = [adafruit_tsl2561.TSL2561(i2c, address=const(0x29)), adafruit_tsl2561.TSL2561(i2c)]
# Print chip info

sched = BackgroundScheduler()
sched.start()

sched.add_job(getLux, 'cron', second='*/5', id='lux')


LED_pin = 10

pins1 = [4, 14, 15, 10]
pins2 = [17, 27, 22, 23]
pins3 = [24, 9, 25, 11]
pins4 = [8, 7, 1, 5]



led = LED(num_pixels, pixel_pin, 0.59, rgb)
ctr = [0, 0, 0, 0]
step1 = step(pins1, 0, 0)
step2 = step(pins2, 0, 0)
step3 = step(pins3, 0, 0)
step4 = step(pins4, 0, 0)

step1.ini()
step2.ini()
step3.ini()
step4.ini()
led.colorChange()
step_arr = [step1, step2, step3, step4]

procs = [Process(target=step1.ctr, args=(step1.act.value,)), Process(target=step2.ctr, args=(step2.act.value,)),
         Process(target=step3.ctr, args=(step3.act.value,)), Process(target=step4.ctr, args=(step4.act.value,))]

try:
    while True:
        # client.loop()
        control(n, ctr, procs)
except KeyboardInterrupt:
    pixels.fill((0, 0, 0))
    pixels.show()