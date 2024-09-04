import RPi.GPIO as GPIO
from time import sleep
import math
import time

# Define the pins connected to the servo motors for the gripper
servo1_pin = 13  # GPIO13 for servo1 (joint 1)
servo2_pin = 19  # GPIO19 for servo2 (joint 2)
gripper_pin = 26 # GPIO26 for gripper
base_pin = 21    # GPIO21 for base rotation

# Define the pins connected to the ultrasonic sensor
TRIG_PIN = 6
ECHO_PIN = 5

# Constants for arm lengths
L1 = 12  # Length of the first arm segment
L2 = 17  # Length of the second arm segment

# Initialize GPIO settings
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)
GPIO.setup(servo1_pin, GPIO.OUT)
GPIO.setup(servo2_pin, GPIO.OUT)
GPIO.setup(gripper_pin, GPIO.OUT)
GPIO.setup(base_pin, GPIO.OUT)

# Define Servo objects
servo1 = GPIO.PWM(servo1_pin, 50)  # 50 Hz frequency
servo2 = GPIO.PWM(servo2_pin, 50)
gripper = GPIO.PWM(gripper_pin, 50)
base = GPIO.PWM(base_pin, 50)

# Initialize PWM for servos
servo1.start(0)
servo2.start(0)
gripper.start(0)
base.start(0)

theta1 = 0
theta2 = 0

def read_distance():
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    sleep(0.00001)
    GPIO.output(TRIG_PIN, GPIO.LOW)

    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()

    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance_cm = pulse_duration * 17150
    return distance_cm

def rotate_base():
    
    step_size = 0.1  # Adjust as needed
    delay = 0.05     # Adjust as needed
        
    current_duty_cycle1 = 2.5
    target_duty_cycle1 = 2.5 + (180 / 18)
    while current_duty_cycle1 != target_duty_cycle1:
        if current_duty_cycle1 < target_duty_cycle1:
            current_duty_cycle1 += step_size
            if current_duty_cycle1 > target_duty_cycle1:
                current_duty_cycle1 = target_duty_cycle1
        else:
            current_duty_cycle1 -= step_size
            if current_duty_cycle1 < target_duty_cycle1:
                current_duty_cycle1 = target_duty_cycle1

        base.ChangeDutyCycle(current_duty_cycle1)
        time.sleep(delay)
    #base.ChangeDutyCycle(12.5)  # Rotate base 180 degrees
    print("Base rotated to place object.")
    sleep(3)  # Wait for rotation to complete

    open_gripper()  # Open the gripper to release the object
    print("Object placed. Gripper is open.")
    sleep(1)

    base.ChangeDutyCycle(2.5)  # Rotate back to initial position
    sleep(3)
    initial_position()

def inv_kinematics(x, y):
    # Calculate distance and angle
    d = math.sqrt(x**2 + y**2)
    print("d",d)
    theta = math.degrees(math.atan2(y, x))
    print("theta", theta)

    if (L1 + L2) >= d and abs(L1 - L2) <= d:
        beta = math.degrees(math.acos((L1**2 + L2**2 - d**2) / (2 * L1 * L2)))
        alpha = math.degrees(math.acos((L1**2 + d**2 - L2**2) / (2 * L1 * d)))

        theta1 = 180-(theta + alpha+ 40)
        theta2 =  abs(beta - 90) 

        # servo2.ChangeDutyCycle(2.5 + (theta2 / 18))
        # time.sleep(1)    
        # servo1.ChangeDutyCycle(2.5 + (theta1 / 18))
        #time.sleep(1)
        #servo2.ChangeDutyCycle(2.5 + (theta2 / 18))
        
         # Define step size and delay for smoother movement
        step_size = 0.1  # Adjust as needed
        delay = 0.05     # Adjust as needed
        
        current_duty_cycle1 = 7.5
        target_duty_cycle1 = 2.5 + (theta1 / 18)
        while current_duty_cycle1 != target_duty_cycle1:
            if current_duty_cycle1 < target_duty_cycle1:
                current_duty_cycle1 += step_size
                if current_duty_cycle1 > target_duty_cycle1:
                    current_duty_cycle1 = target_duty_cycle1
            else:
                current_duty_cycle1 -= step_size
                if current_duty_cycle1 < target_duty_cycle1:
                    current_duty_cycle1 = target_duty_cycle1

            servo1.ChangeDutyCycle(current_duty_cycle1)
            time.sleep(delay)

        # Gradually change the duty cycle for servo2
        current_duty_cycle2 = 2.5
        target_duty_cycle2 = 2.5 + (theta2 / 18)
        while current_duty_cycle2 != target_duty_cycle2:
            if current_duty_cycle2 < target_duty_cycle2:
                current_duty_cycle2 += step_size
                if current_duty_cycle2 > target_duty_cycle2:
                    current_duty_cycle2 = target_duty_cycle2
            else:
                current_duty_cycle2 -= step_size
                if current_duty_cycle2 < target_duty_cycle2:
                    current_duty_cycle2 = target_duty_cycle2

            servo2.ChangeDutyCycle(current_duty_cycle2)
            time.sleep(delay)
        print("Arm moved to position.")
        print("theta1",theta1)
        print("theta2",theta2)
        sleep(2)

        # Close the gripper once the arm has reached the target position
        close_gripper()
        
        sleep(2)
        
        step_size = 0.1  # Adjust as needed
        delay = 0.05     # Adjust as needed
        
        current_duty_cycle1 = 2.5+(theta1/18)
        target_duty_cycle1 = 7.5
        while current_duty_cycle1 != target_duty_cycle1:
            if current_duty_cycle1 < target_duty_cycle1:
                current_duty_cycle1 += step_size
                if current_duty_cycle1 > target_duty_cycle1:
                    current_duty_cycle1 = target_duty_cycle1
            else:
                current_duty_cycle1 -= step_size
                if current_duty_cycle1 < target_duty_cycle1:
                    current_duty_cycle1 = target_duty_cycle1

            servo1.ChangeDutyCycle(current_duty_cycle1)
            time.sleep(delay)
        
        
        #servo1.ChangeDutyCycle(7.5)
        servo2.ChangeDutyCycle(2.5)
        
        sleep(1)
        rotate_base()
    else:
        print("Position out of reach.")
        initial_position()

def initial_position():
    servo1.ChangeDutyCycle(7.5)  # Center position for servo1
    servo2.ChangeDutyCycle(2.5)  # Center position for servo2
    base.ChangeDutyCycle(2.5)     # Center position for base
    open_gripper()

def open_gripper():
    gripper.ChangeDutyCycle(12)  # Open gripper
    sleep(1)
    print("Gripper is open.")

def close_gripper():
    gripper.ChangeDutyCycle(2.5)  # Close gripper
    sleep(1)
    print("Gripper is closed.")

try:
    # Move servos to initial position and open the gripper
    initial_position()
    # Read distance from ultrasonic sensor
    dist = read_distance()
    print("Distance:", dist, "cm")

    # Coordinates of the target point
    x = dist
    y = 12

    # Compute inverse kinematics for the given target point
    inv_kinematics(x, y)

    # Wait for the arm to reach the target position

        

except KeyboardInterrupt:
    print("Program stopped.")
    GPIO.cleanup()        