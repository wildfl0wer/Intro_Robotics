"""
Description:
This program simulates traffic lights at an intersection using LEDs.
 
The traffic light sequence starts with red for the horizontal lights and green for the vertical lights.
- Horizontal Red: 5 seconds      - Vertical Green: 5 seconds
- Horizontal Red: 2 seconds      - Vertical Yellow: 2 seconds
- Horizontal Red: 1 second       - Vertical Red: 1 seconds

The sequence is then repeated for the vertical lights.
- Horizontal Green: 5 seconds      - Vertical Red: 5 seconds
- Horizontal Yellow: 2 seconds     - Vertical Red: 2 seconds
- Horizontal Red: 1 second         - Vertical Red: 1 seconds

After the first cycle of sequence, the total runtime is as follows:
    - Red light: 9 seconds
    - Green light: 5 seconds
    - Yellow light: 2 seconds
"""

import RPi.GPIO as GPIO             # Import Rasberry Pi GPIO library
import time

# Sets up GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)             # Ignores warnings

#Variables: LED pins
VERTICAL_LEDS = {
    "RED": 25,
    "YELLOW": 12,
    "GREEN": 16
}

HORIZONTAL_LEDS = {
    "RED": 6,
    "YELLOW": 13,
    "GREEN": 26
}

ALL_LEDS = list(VERTICAL_LEDS.values()) + list(HORIZONTAL_LEDS.values())

# Sets up GPIO outputs for all LED pins
GPIO.setup(ALL_LEDS, GPIO.OUT)


def lights_off(list_led_pins):
    """
    Turns off all LEDs in given list of pins

    Args:
        :param list_led_pins (list): List of GPIO pins of LEDs to turn off 
    """
    for pin in list_led_pins:
        GPIO.output(pin, GPIO.LOW)
    

def light_on(led_name, led_dictionary):
    """
    Turns on LED specified by led_name and turns off other LEDs in same list

    Args:
        :param led_name (str): Descriptive name of LED to turn on
        :param led_dictionary (dict): Dictionary containing LED names and
            corresponding pin numbers
    """
    # Checks if LED name is in LED dictionary
    if led_name not in led_dictionary:
        print("Invalid LED name")
        return

    # Gets pin number from LED dictionary
    led_pin = led_dictionary[led_name]

    # Turns off other LEDs in the same list
    lights_off(list(led_dictionary.values()))

    # Turns on specified LED
    GPIO.output(led_pin, GPIO.HIGH)


try:
    lights_off(ALL_LEDS)    # Ensures all LEDs are off before starting sequence
    
    while True:
        light_on("GREEN", VERTICAL_LEDS)
        light_on("RED", HORIZONTAL_LEDS)
        time.sleep(5)

        light_on("YELLOW", VERTICAL_LEDS)
        time.sleep(2)

        light_on("RED", VERTICAL_LEDS)
        time.sleep(1)

        light_on("RED", VERTICAL_LEDS)
        light_on("GREEN", HORIZONTAL_LEDS)
        time.sleep(5)

        light_on("YELLOW", HORIZONTAL_LEDS)
        time.sleep(2)

        light_on("RED", HORIZONTAL_LEDS)
        time.sleep(1)

except KeyboardInterrupt:    # When the console is stopped (CTRL + C)
    GPIO.cleanup()           # Cleans up GPIO
