import time
import RPi.GPIO as GPIO
#from multiprocessing import Process, Value
#import keyboard
import paho.mqtt.client as mqtt
import board  # Simple test for NeoPixels on Raspberry Pi
import neopixel
import busio
import adafruit_tsl2561
from micropython import const
from apscheduler.schedulers.background import BackgroundScheduler
from apscheduler.jobstores.base import JobLookupError
#import VL53L0X
import serial


# 시리얼 통신에 관한 설정

"""
port = '/dev/ttyACM0'
brate = 9600
auto = False
seri = serial.Serial(port, baudrate = brate, timeout = None)
print(seri.name)
"""

# LED의 초기 색상과 밝기 설정
# 커튼의 초기 단계 설정
n = 0
rgb = [0, 230, 50]
brightness = 0

# mqtt 연결 설정
def on_connect(client, userdata, rc, properties=None):
    print("Connected with result code " + str(rc))
    client.subscribe("ctn/step")
    client.subscribe("auto/ctr")
    client.subscribe("led/color")
    client.subscribe("led/bright")

# mqtt 토픽에 의한 행동 설정
def on_message(client, userdata, msg):
    global n
    global led

    if msg.topic == "ctn/step":
        n = int(msg.payload)
    elif msg.topic == "led/color":
        led.rgb = map(int, msg.payload.decode("utf-8").split('|'))
        led.colorChange()
    elif msg.topic == "led/bright":
        led.bright = float(msg.payload) * 1.25
        print("asdfasfsdfBifdfasdfsdf")
        led.setBright()
    elif msg.topic == "auto/ctr":
        if msg.payload == "0":
            auto = False
        else:
            auto = True
        #auto_detection()
    print(msg.topic + " " + str(msg.payload))

client = mqtt.Client(client_id="asdf")
client.on_connect = on_connect
client.on_message = on_message

client.connect("192.168.120.158", 1883, 60)
client.loop_start()

# 센서 사용에 팔요한 GPIO 설정
GPIO.setmode(GPIO.BCM)
# 네오픽셀 Led는 D10, D12, D18 or D21에서만 작동
# LED GPIO 18번 핀으로 설정
pixel_pin = board.D18
# 픽셀 수 (24구)
num_pixels = 24
#색상 순서 Green, Red, Blue
ORDER = neopixel.GRB

# LED의 기본설정
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
        

# curtain
class step:
    def __init__(self, Pins, status):
        self.StepCount = 4
        self.StepCounter = 0
        self.Pins = Pins
        self.status = status

    def ini(self):
        for pin in self.Pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, False)

# motor의 up 시퀀스
    def up(self):
#       up = [[0, 0, 0, 1], [0, 0, 1, 0], [0, 1, 0, 0], [1, 0, 0, 0]]

        up = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        try:
            if self.status > 0:
                for pin in range(0, 4):
                    xpin = self.Pins[pin]
                    if up[self.status % 4][pin] != 0:
                        GPIO.output(xpin, True)
                    else:
                        GPIO.output(xpin, False)
               	self.status -= 1
            time.sleep(0.001)
        except KeyboardInterrupt:
            print()

#       print(self.status.value)

# motor 의 down 시퀀스
    def down(self):
        down = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        try:
            if self.status < 10000:
                for pin in range(0, 4):
                    xpin = self.Pins[pin]
                    if down[self.status % 4][pin] != 0:
                        GPIO.output(xpin, True)
                    else:
                        GPIO.output(xpin, False)
                self.status += 1
            time.sleep(0.001)
        except KeyboardInterrupt:
            print()
        # print(self.status.value)

    def ctr(self, control):
        if control == 1:
            self.down()
        else:
            self.up()
# step class final

def control(steps):
    if steps == 0:
        for i in range(0, 4):
            step_arr[i].ctr(0)
    else:
        for i in range(0, 4):
            if i < steps:
                step_arr[i].ctr(1)
            else:
                step_arr[i].ctr(0)

# get Lux from sensor
# 자동 감지 함수, getlux(아두이노)
"""
def auto_detection():
        a = 0
        if(auto):
                seri.write(auto.encode())
                a=1
                while a:
                        if seri.in_waiting !=0 :
                                content = seri.readline()
                                print(content[:2].decode())
                                a = 0
getLux():

        a = 0
        seri.write(lux.encode())
        a = 1
        while a:
                if seri.in_waiting != 0:
                        content = seri.readline()
                        print(content[:2].decode())
                        a = 0
"""

def getLux():
    now = time.localtime()
    date = "%04d-%02d-%02d %02d:%02d:%02d" % (now.tm_year, now.tm_mon, now.tm_mday, now.tm_hour, now.tm_min, now.tm_sec)
    lux_data = []
    for i in tsl:
        lux = None
        while lux == None:
            #print("Chip ID = {}".format(i.chip_id))

            # Enable the light sensor
            i.enabled = True
            time.sleep(1)

            # Set gain 0=1x, 1=16x
            i.gain = 1

            # Set integration time (0=13.7ms, 1=101ms, 2=402ms, or 3=manual)
            i.integration_time = 1

            #print("Getting readings...")

            # Get raw (luminosity) readings individually    

            # Get raw (luminosity) readings using tuple unpacking
            # broadband, infrared = tsl.luminosity

            # Get computed lux value (tsl.lux can return None or a float)

            lux = i.lux
            print(lux)
            i.enabled = False    
        lux = str(lux)
        lux_data.append(lux)
        
    
        # Disble the light sensor (to save power)
    lux_data = date + "|" + "|".join(lux_data)
    print("--lux date, lux--")
    
    print(lux_data)
    client.publish("Database/bright/save", lux_data)
    
    
        

# ---Lightsensor---
# Create the I2C bus
# 조도센서 i2c bus 연결 설정
i2c = busio.I2C(board.SCL, board.SDA)
print(i2c.scan())
print(board.SCL, board.SDA)

# 2개의 조도센서 설정
# Create the TSL2561 instance, passing in the I2C bus
tsl = [adafruit_tsl2561.TSL2561(i2c, address=const(0x29)), adafruit_tsl2561.TSL2561(i2c)]
print(tsl)

# 스케쥴러 사용
sched = BackgroundScheduler()
sched.start()

sched.add_job(getLux, 'cron', second='*/5', id='lux')

# motor가 연결된 핀 리스트
pins1 = [4, 14, 15, 10]
pins2 = [17, 27, 22, 23]
pins3 = [24, 9, 25, 11]
pins4 = [8, 7, 1, 5]

# led 객체 생성 객체 실행
led = LED(num_pixels, pixel_pin, 0, rgb)

# 모터 객체 생성 객체 실행
step1 = step(pins1, 0)
step2 = step(pins2, 0)
step3 = step(pins3, 0)
step4 = step(pins4, 0)

step1.ini()
step2.ini()
step3.ini()
step4.ini()
led.colorChange()

# step_arr 상태 확인 
step_arr = [step1, step2, step3, step4]

try:
    while True:
        # mqtt client connect only network
        client.loop()
        control(n)
        getLux()
        print([i.status for i in step_arr])
        
except KeyboardInterrupt:
    pixels.fill((0, 0, 0))
    pixels.show()
