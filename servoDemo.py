import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

servoPin = 23

GPIO.setup(servoPin, GPIO.OUT)

pwm = GPIO.PWM(servoPin, 50)

pwm.start(0)

def servo(angle):
    angle = max(0, min(180, angle))
    duty_cycle = (angle/180) * 10 + 2.5
    GPIO.output(servoPin, True)
    pwm.ChangeDutyCycle(duty_cycle)
    GPIO.output(servoPin, False)

while True:
    angle = int(input("Enter a value: "))
    servo(angle)
    
