"""
Description:
This program controls a servo motor's movement according to which buttons the
user presses.

Default servo position: 90 degrees.
Left button press: Moves servo to 180 degrees.
Right button press: Moves servo to 0 degrees.
Simultaneous press of both buttons initiates servo sweep from 0 to 180 degrees
at 10-degree intervals every two seconds.

Additionally, when both buttons are pressed simultaneously, the number of
completed sweeps is displayed using four LEDs in binary.
"""

import RPi.GPIO as GPIO                 # Import Rasberry Pi GPIO library
import time

# Constants: Pins
BUTTON_LEFT = 12
BUTTON_RIGHT = 16
SERVO_PIN = 23

# Constants: LEDs that represent binary bits
LED_PINS = [5, 6, 13, 26]              # [8, 4, 2, 1]

# Constants: Servo Positions
SERVO_POSITION_DEFAULT = 90
SERVO_POSITION_LEFT_BUTTON = 180
SERVO_POSITION_RIGHT_BUTTON = 0

# Sets up GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)                # Ignores warnings
GPIO.setup([BUTTON_LEFT, BUTTON_RIGHT], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup([SERVO_PIN] + LED_PINS, GPIO.OUT)

# Sets up PWM for servo
servo_pwm = GPIO.PWM(SERVO_PIN, 50)    # 50 Hz (20 miliseconds PWM period)
servo_pwm.start(0)                     # Starts PWM with duty cycle 0


def move_servo(angle):
    """
    Move the servo motor to a specified angle.
    
    Modification of code provided by Prof. J. Aparicio

    Args:
        :param angle (int): The angle to which the servo motor should be
            moved. Should be in the range from 0 to 180 degrees.
    
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
    GPIO.output(SERVO_PIN, True)            # GPIO to HIGH
    servo_pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.3)                         # Time for servo to move to position
    GPIO.output(SERVO_PIN, False)           # GPIO to LOW
    servo_pwm.ChangeDutyCycle(0)


def sweep_servo():
    """
    Sweeps the servo motor from 0 to 180 degrees in increments of 10 degrees
    with a time delay of 2 seconds between each angle increment.
    """
    
    for angle in range(0, 181, 10): 
        move_servo(angle)
        time.sleep(2)              # Time between increments


def update_leds(decimal_count):
    """
    Updates the state of LEDs based on the binary representation
    of a decimal number.

    Args:
        :param decimal_count (int): The decimal number to be
            represented in binary format.

    Description:
        This function converts the given decimal number into a 4-bit
        binary string, then iterates over LED pins and corresponding
        bits in the binary string. Each LED pin's state is updated
        based on the corresponding bit value. If the bit is 1, the
        LED pin is set to HIGH (ON), otherwise it is set to LOW
        (OFF).
    """

    # Converts integer to binary stiring
    binary_string = format(decimal_count, "04b")

    for pin, bit in zip(LED_PINS, binary_string):
        GPIO.output(pin, int(bit))
    

try:
    # Initializes counter for simultaneous button presses
    count_both_buttons_press = 0
    
    # Sets servo and LEDs to default status
    move_servo(SERVO_POSITION_DEFAULT)
    update_leds(count_both_buttons_press)

    while True:
        pressed_left_button = GPIO.input(BUTTON_LEFT) == GPIO.HIGH
        pressed_right_button = GPIO.input(BUTTON_RIGHT) == GPIO.HIGH

        if pressed_left_button and pressed_right_button:
            sweep_servo()
            count_both_buttons_press += 1
            update_leds(count_both_buttons_press)

        if pressed_left_button and not pressed_right_button:
            move_servo(SERVO_POSITION_LEFT_BUTTON)

        if pressed_right_button and not pressed_left_button:
            move_servo(SERVO_POSITION_RIGHT_BUTTON)

        move_servo(SERVO_POSITION_DEFAULT)
        time.sleep(0.2)      # Sets delay to help with servo twitch
            
except KeyboardInterrupt:    # (CTRL + C)
    servo_pwm.stop()
    GPIO.cleanup()
