import RPi.GPIO as GPIO
import time

IN1 = 5
IN2 = 6
IN3 = 13
IN4 = 19

GPIO.setmode(GPIO.BCM)
GPIO.setup([IN1, IN2, IN3, IN4], GPIO.OUT)


def backward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)


def forward():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)


def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)


while True:
    user_input = input("1 - Forward, 2 - Backward, 3 - Stop: ")
    if user_input == "1":
        forward()
    elif user_input == "2":
        backward()
    elif user_input == "3":
        stop()
    else:
        print("Invalid input")
