import time
import RPi.GPIO as GPIO
from multiprocessing import Process, Value
import keyboard
import paho.mqtt.client as mqtt
import board # Simple test for NeoPixels on Raspberry Pi
import neopixel

n = 0 

def on_connect(client, userdata, rc, properties=None):
    print("Connected with result code " + str(rc))
    client.subscribe("Curtain/ctr")


def on_message(client, userdata, msg):
    global n
    if msg.topic == "Curtain/ctr":
        n = int(msg.payload)
    print(msg.topic + " " + str(msg.payload))


client = mqtt.Client(client_id="asdf")
client.on_connect = on_connect
client.on_message = on_message

client.connect("192.168.175.248", 1883, 60)

client.loop_start()
"""
class LED:
    GPIO.setmode(GPIO.BCM)

    # Choose an open pin connected to the Data In of the NeoPixel strip, i.e. board.D18
    # NeoPixels must be connected to D10, D12, D18 or D21 to work.
    pixel_pin = board.D18
     
    # The number of NeoPixels
    num_pixels = 12
     
    # The order of the pixel colors - RGB or GRB. Some NeoPixels have red and green reversed!
    # For RGBW NeoPixels, simply change the ORDER to RGBW or GRBW.
    ORDER = neopixel.GRB
     
    pixels = neopixel.NeoPixel(
        pixel_pin, num_pixels, brightness=0.09, auto_write=False, pixel_order=ORDER
    )
     
     
    def wheel(pos):
        # Input a value 0 to 255 to get a color value.
        # The colours are a transition r - g - b - back to r.
        if pos < 0 or pos > 255:
            r = g = b = 0
        elif pos < 85:
            r = int(pos * 3)
            g = int(255 - pos * 3)
            b = 0
        elif pos < 170:
            pos -= 85
            r = int(255 - pos * 3)
            g = 0
            b = int(pos * 3)
        else:
            pos -= 170
            r = 0
            g = int(pos * 3)
            b = int(255 - pos * 3)
        return (r, g, b) if ORDER in (neopixel.RGB, neopixel.GRB) else (r, g, b, 0)
     
     
    def rainbow_cycle(wait):
        for j in range(255):
            for i in range(num_pixels):
                pixel_index = (i * 256 // num_pixels) + j
                pixels[i] = wheel(pixel_index & 255)
            pixels.show()
            time.sleep(wait)
     
    try:
        while True:
            # Comment this line out if you have RGBW/GRBW NeoPixels
            pixels.fill((255, 0, 0))
            # Uncomment this line if you have RGBW/GRBW NeoPixels
            # pixels.fill((255, 0, 0, 0))
            pixels.show()
            time.sleep(1)
         
            # Comment this line out if you have RGBW/GRBW NeoPixels
            pixels.fill((0, 255, 0))
            # Uncomment this line if you have RGBW/GRBW NeoPixels
            # pixels.fill((0, 255, 0, 0))
            pixels.show()
            time.sleep(1)
         
            # Comment this line out if you have RGBW/GRBW NeoPixels
            pixels.fill((0, 0, 255))
            # Uncomment this line if you have RGBW/GRBW NeoPixels
            # pixels.fill((0, 0, 255, 0))
            pixels.show()
            time.sleep(1)
         
            rainbow_cycle(0.001)  # rainbow cycle with 1ms delay per step
    except KeyboardInterrupt:
        pixels.fill((0, 0, 0))
        pixels.show()
"""
#curtain
class step:

     def __init__(self, Pins, status, act):
         self.Pins = Pins
         self.status = Value('i', status)
         self.act = Value('i', act)
         
     def ini(self):
        self.StepCounter = 0
        self.StepCount = 4
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
         #print(self.status.value)

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
         #print(self.status.value)

     def ctr(self, control):
         if control == 1:
             self.down()
         else:
             self.up()


def control(steps, ctr, procs):
     if steps == 0:
         for i in range(len(ctr)):
             ctr[i] = 0
             procs[i] = Process(target=step_arr[i].ctr, args=(ctr[i], ))
             procs[i].start()
     else:
         for i in range(len(ctr)):
             if i < steps:
                 ctr[i] = 1
             else:
                 ctr[i] = 0
             procs[i] = Process(target=step_arr[i].ctr, args=(ctr[i], ))
             procs[i].start()


pins1 = [26, 19, 13, 6]
pins2 = [21, 20, 16, 12]
pins3 = [24, 23, 18, 15]
pins4 = [1, 7, 8, 25]
ctr = [0, 0, 0, 0]
step1 = step(pins1, 0, 0)
step2 = step(pins2, 0, 0)
step3 = step(pins3, 0, 0)
step4 = step(pins4, 0, 0)
step1.ini()
step2.ini()
step3.ini()
step4.ini()
step_arr = [step1, step2, step3, step4]

procs = [Process(target=step1.ctr, args=(step1.act.value, )), Process(target=step2.ctr, args=(step2.act.value, )),
Process(target=step3.ctr, args=(step3.act.value, )), Process(target=step4.ctr, args=(step4.act.value, ))]
while True:
    #client.loop()
    if keyboard.is_pressed("0"):
         n = 0
    elif keyboard.is_pressed("1"):
         n = 1
    elif keyboard.is_pressed("2"):
         n = 2
    elif keyboard.is_pressed("3"):
         n = 3
    elif keyboard.is_pressed("4"):
         n = 4
    else:
         control(n, ctr, procs)
    print(ctr)
                                                        