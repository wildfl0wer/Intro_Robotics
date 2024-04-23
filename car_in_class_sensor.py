import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)                # Ignores warnings

TRIG = 16
ECHO = 20

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)


def distance():     # returns distance in centimeters
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    start_time = time.time()
    while GPIO.input(ECHO) == 0 and time.time() - start_time < 0.1:
        pulse_start = time.time()

    while GPIO.input(ECHO) == 1 and  time.time() - start_time < 0.1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    print("TEST:", (pulse_duration * 34300) / 2)
    return (pulse_duration * 34300) / 2


try:
    while True:
        dist = int(distance())
        print(dist)
        time.sleep(1)
        
except KeyboardInterrupt:    # (CTRL + C)
    GPIO.cleanup()
