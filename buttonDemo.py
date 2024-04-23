import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

buttonPin = 16
ledPin = 6

GPIO.setup(buttonPin, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
GPIO.setup(ledPin, GPIO.OUT)

while True:
    if GPIO.input(buttonPin) == GPIO.HIGH:
        GPIO.output(ledPin, GPIO.HIGH)
    else:
        GPIO.output(ledPin, GPIO.LOW)
