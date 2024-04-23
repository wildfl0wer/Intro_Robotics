import RPi.GPIO as GPIO
import time
#from time import time

GPIO.setmode(GPIO.BCM)

redLED = 16
greenLED = 20
yellowLED = 21

GPIO.setup(16, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)

try:
    while True:
        GPIO.output(redLED, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(redLED, GPIO.LOW)
        
        GPIO.output(greenLED, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(greenLED, GPIO.LOW)

        GPIO.output(yellowLED, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(yellowLED, GPIO.LOW)
        time.sleep(1)
except KeyboardInterrupt:
    GPIO.cleanup()

