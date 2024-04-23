import RPi.GPIO as GPIO
import time

# Constants: Servos
STEERING_SERVO = 26
SENSOR_SERVO = 12

# Constants: Servo Positions
STEERING_SERVO_POSITION_DEFAULT = 55
STEERING_SERVO_POSITION_LEFT = 25
STEERING_SERVO_POSITION_RIGHT = 85
SENSOR_SERVO_POSITION_DEFAULT = 90

# Sets up GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)                # Ignores warnings
GPIO.setup([STEERING_SERVO, SENSOR_SERVO], GPIO.OUT)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Sets up PWM for servo
steering_pwm = GPIO.PWM(STEERING_SERVO, 50)    # 50 Hz (20 miliseconds PWM period)
sensor_pwm = GPIO.PWM(SENSOR_SERVO, 50)    # 50 Hz (20 miliseconds PWM period)
steering_pwm.start(0)
sensor_pwm.start(0)


def servo(angle):
    angle = max(0, min(180, angle))
    duty_cycle = (angle/180) * 10 + 2.5
    GPIO.output(servoPin, True)
    pwm.ChangeDutyCycle(duty_cycle)
    GPIO.output(servoPin, False)


def move_servo(angle, servo_pin, servo_pwm):
    """
    Move the servo motor to a specified angle.
    
    Modification of code provided by Prof. J. Aparicio

    Args:
        :param angle (int): The angle to which the servo motor should be
            moved. Should be in the range from 0 to 180 degrees.
        :param servo_pin (int): The pin of the servo to move.
    
    Description:
        This function calculates the duty cycle required to move
        the servo motor to the specified angle. It sets the GPIO
        pin connected to the servo motor to HIGH, adjusts the duty
        cycle of the PWM signal accordingly, introduces a small
        delay (0.3 seconds) to allow the servo to move to the
        specified position, sets the GPIO pin to LOW, and resets
        the duty cycle to 0.
    """
    
    duty_cycle = (angle/180) * 10 + 2.5
    GPIO.output(servo_pin, True)            # GPIO to HIGH
    servo_pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.3)                         # Time for servo to move to position
    GPIO.output(servo_pin, False)           # GPIO to LOW
    servo_pwm.ChangeDutyCycle(0)


def sweep_servo(servo_pin):
    """
    Sweeps the servo motor from 0 to 180 degrees in increments of 10 degrees
    with a time delay of 2 seconds between each angle increment.
    """
    
    for angle in range(0, 181, 10): 
        move_servo(angle, servo_pin)
        time.sleep(.1)              # Time between increments


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
    print("TEST:", int((pulse_duration * 34300) / 2))
    return (pulse_duration * 34300) / 2


def detect_obstacle():
    if int(distance()) < 50:
        stop()


try:
    # Sets servo and LEDs to default status
    move_servo(STEERING_SERVO_POSITION_DEFAULT, STEERING_SERVO, steering_pwm)
    move_servo(SENSOR_SERVO_POSITION_DEFAULT, SENSOR_SERVO, sensor_pwm)

    while True:
        dist = int(distance())

        if dist > 50:
            Print("Moving forward.")
        else:
            detect_obstacle()
            print("Obstacle detected!")
            direction = find_furthest_direction()
            print("DIRECTION:", direction)
            if direction > 50:
                move_direction(direction)
            else:
                backward()
                time.sleep(1)
                stop

                continue
    
