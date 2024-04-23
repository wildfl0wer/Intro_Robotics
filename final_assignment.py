"""
Description:
This program controls a car-like robot equipped with ultrasonic sensors and servo motors using a Raspberry Pi. The robot's objective is
to autonomously navigate its environment while avoiding obstacles.

The program continuously monitors the distance from obstacles. When an obstacle is detected within a certain range, the robot halts,
identifies the safest direction to maneuver by sweeping the sensor from the right to left, and proceeds to either turn or move backward,
depending on the available space. If no obstacles are within the range, the robot continues moving forward.

"""

import RPi.GPIO as GPIO                 # Import Rasberry Pi GPIO library
import time

# Constants: Motor Pins
IN1, IN2, IN3, IN4 = 5, 6, 13, 19

# Constants: Sensor Pins
TRIG, ECHO = 16, 20

# Constants: Servo Pins
STEERING_SERVO, SENSOR_SERVO = 26, 12

# Constants: Servo Positions
STEERING_SERVO_POSITION_DEFAULT = 55
STEERING_SERVO_POSITION_LEFT = 25
STEERING_SERVO_POSITION_RIGHT = 85

SENSOR_SERVO_POSITION_DEFAULT = 90
SENSOR_SERVO_POSITION_MIN = 20
SENSOR_SERVO_POSITION_MAX = 160
SENSOR_SERVO_ANGLE_INCREMENT = 20


# Constant: Minimum distance between car & obstacle
SAFE_DISTANCE = 50

try:
    # Sets up GPIO mode
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)                # Ignores warnings
    GPIO.setup([IN1, IN2, IN3, IN4], GPIO.OUT)
    GPIO.setup([STEERING_SERVO, SENSOR_SERVO], GPIO.OUT)
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)
    
    # Sets up PWM for servos
    steering_pwm = GPIO.PWM(STEERING_SERVO, 50)    # 50 Hz (20 miliseconds PWM period)
    sensor_pwm = GPIO.PWM(SENSOR_SERVO, 50)    # 50 Hz (20 miliseconds PWM period)
    steering_pwm.start(0)
    sensor_pwm.start(0)
    
    
    def move_backward():
        """
        Move the car backward by activating appropriate motor pins.
        """
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
    
    
    def move_forward():
        """
        Move the car forward by activating appropriate motor pins.
        """
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
    
    
    def stop_motors():
        """
        Stop all motors by deactivating all motor pins.
        """
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)

    
    def move_servo(angle, servo_pin, servo_pwm):
        """
        Move the servo motor to a specified angle.
    
        Args:
            angle (int): The angle to which the servo motor should be moved.
                Should be in the range from 0 to 180 degrees.
            servo_pin (int): The pin of the servo to move.
            servo_pwm: The PWM object associated with the servo.
        
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

    
    def determine_distance():
        """
        Determine the distance from an obstacle using ultrasonic sensor.
    
        Returns:
            float: The distance from the obstacle in centimeters.
            
        Description:
            This function utilizes an ultrasonic sensor to measure the distance from
            an obstacle. It sends a short ultrasonic pulse to the sensor, measures the
            time it takes for the pulse to bounce back, and calculates the distance
            based on the time taken and the speed of sound. The calculated distance is
            returned in centimeters.
        """
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        pulse_start = time.time()
        pulse_end = pulse_start  # Initializes pulse_end to avoid NameError if not set within loops
    
        start_time = time.time()
        while GPIO.input(ECHO) == 0 and time.time() - start_time < 0.1:
            pulse_start = time.time()
    
        while GPIO.input(ECHO) == 1 and  time.time() - start_time < 0.1:
            pulse_end = time.time()

        # Check if pulse_start and pulse_end were not updated within the loops
        if pulse_start == pulse_end:
            raise RuntimeError("Echo signal not received within time limit")
    
        pulse_duration = pulse_end - pulse_start
        #print("Centimeters:", int((pulse_duration * 34300) / 2))
        return (pulse_duration * 34300) / 2
    
    
    def find_furthest_direction():
        """
        Find the direction with the furthest distance from obstacles.
    
        Returns:
            int: The angle corresponding to the direction with the furthest distance.
            
        Description:
            This function sweeps the sensor servo through the specified range of angles
            (from SENSOR_SERVO_POSITION_MIN to SENSOR_SERVO_POSITION_MAX) with the given
            angle increment (SENSOR_SERVO_ANGLE_INCREMENT). It measures the distance for
            each angle and determines the direction with the furthest distance from obstacles.
            It returns the angle corresponding to that direction. If no safe direction is found,
            it returns 0.
        """
        max_distance = 0
        best_direction = None
    
        # Sweep the sensor servo through the provided range
        for angle in range(SENSOR_SERVO_POSITION_MIN,
                           SENSOR_SERVO_POSITION_MAX + 1,
                           SENSOR_SERVO_ANGLE_INCREMENT):
            move_servo(angle, SENSOR_SERVO, sensor_pwm)
            time.sleep(0.1)
            dist = int(determine_distance())
            
            if dist > max_distance:
                max_distance = dist
                print("Current BEST DIRECTION:", best_direction)
                best_direction = angle

            # Print the distance for each angle
            print("Angle:", angle, "Distance:", dist)
            print("Max distance:", max_distance)
    
        # Reset the sensor to default position
        move_servo(SENSOR_SERVO_POSITION_DEFAULT, SENSOR_SERVO, sensor_pwm)
        time.sleep(0.1)
    
        if max_distance <= SAFE_DISTANCE:
            return 0

        if best_direction > SENSOR_SERVO_POSITION_DEFAULT:
            best_direction = STEERING_SERVO_POSITION_LEFT
            print("I'm going left!", best_direction)
        elif best_direction < SENSOR_SERVO_POSITION_DEFAULT:
            best_direction = STEERING_SERVO_POSITION_RIGHT
            print("I'm going right!", best_direction)
        else:
            best_direction = 0
            print("I'm going backwards!", best_direction)
        print("Best direction:", best_direction)
        
        return best_direction
    
    
    def turn_car(angle):
        """
        Turn the car to a specified angle.
    
        Args:
            angle (int): The angle to which the car should be turned.
        Description:
            This function turns the car by first moving the steering servo to the specified angle,
            then moving the car forward for a fixed duration (1 second) to execute the turn, and
            finally stopping the car and resetting the steering servo to its default position.
        """
        # Move the steering servo to the specified angle
        move_servo(angle, STEERING_SERVO, steering_pwm)
        time.sleep(0.1)
    
        # Move the car forward
        move_forward()
        time.sleep(1)
        
        stop_motors()
        time.sleep(0.1)
        
        move_servo(STEERING_SERVO_POSITION_DEFAULT, STEERING_SERVO, steering_pwm)
        time.sleep(0.1)
        
    
    try:
        # Sets servo to default status
        move_servo(STEERING_SERVO_POSITION_DEFAULT, STEERING_SERVO, steering_pwm)
        move_servo(SENSOR_SERVO_POSITION_DEFAULT, SENSOR_SERVO, sensor_pwm)

        while True:
            time.sleep(0.1)
            distance = int(determine_distance())
    
            if distance > SAFE_DISTANCE:
                move_forward()
                #print("Forward!")
            else:
                stop_motors()
                print("Obstacle detected!", distance)
                
                direction = find_furthest_direction()
                if direction == STEERING_SERVO_POSITION_LEFT:
                    print("DIRECTION: LEFT", direction)
                if direction == STEERING_SERVO_POSITION_RIGHT:
                    print("DIRECTION: RIGHT", direction)
                    
                time.sleep(0.1)
                new_distance = int(determine_distance())
                
                if new_distance > SAFE_DISTANCE:
                    turn_car(direction)
                    print("I am turning!", direction, "CM", new_distance)
                else:
                    move_backward()
                    time.sleep(1)
                    print("I am going backwards!", direction, "CM", new_distance)
                
    except KeyboardInterrupt:    # (CTRL + C)
        stop_motors()
        steering_pwm.stop()
        sensor_pwm.stop()
        GPIO.cleanup()

except Exception as e:
    print("An error occurred:", str(e))
    # Handle the error gracefully, e.g., cleanup GPIO resources
    GPIO.cleanup()
