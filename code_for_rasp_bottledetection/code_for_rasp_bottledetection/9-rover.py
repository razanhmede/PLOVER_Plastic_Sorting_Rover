import RPi.GPIO as GPIO
import time
import cv2
import math
from time import sleep

# Define GPIO pins for rover movement
ENA = 17  # Enable pin for Motor A
IN1 = 27  # Control pin 1 for Motor A
IN2 = 22  # Control pin 2 for Motor A
ENB = 18  # Enable pin for Motor B
IN3 = 23  # Control pin 1 for Motor B
IN4 = 24  # Control pin 2 for Motor B

# Define the pins connected to the ultrasonic sensor
TRIG_PIN = 6
ECHO_PIN = 5

# Define the pins connected to the servo motors for the gripper
servo1_pin = 13  # GPIO13 for servo1 (joint 1)
servo2_pin = 19  # GPIO19 for servo2 (joint 2)
gripper_pin = 26 # GPIO26 for gripper
base_pin = 21    # GPIO21 for base rotation

# Constants for arm lengths
L1 = 12  # Length of the first arm segment
L2 = 17  # Length of the second arm segment

# Initialize GPIO settings
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)
GPIO.setup(servo1_pin, GPIO.OUT)
GPIO.setup(servo2_pin, GPIO.OUT)
GPIO.setup(gripper_pin, GPIO.OUT)
GPIO.setup(base_pin, GPIO.OUT)

# Initialize PWM for servo motors
servo1 = GPIO.PWM(servo1_pin, 50)  # 50 Hz frequency
servo2 = GPIO.PWM(servo2_pin, 50)
gripper = GPIO.PWM(gripper_pin, 50)
base = GPIO.PWM(base_pin, 50)

servo1.start(0)
servo2.start(0)
gripper.start(0)
base.start(0)

# Function to move the rover forward
def move_forward(speed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_a.start(speed)
    pwm_b.start(speed)

# Function to move the rover backward
def move_backward(speed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_a.start(speed)
    pwm_b.start(speed)

# Function to turn the rover left
def turn_left(speed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_a.start(speed)
    pwm_b.start(speed)

# Function to turn the rover right
def turn_right(speed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_a.start(speed)
    pwm_b.start(speed)

# Function to brake the rover
def brake_motor():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_a.stop()
    pwm_b.stop()
    
    
pwm_frequency = 1000
pwm_a = GPIO.PWM(ENA, pwm_frequency)
pwm_b = GPIO.PWM(ENB, pwm_frequency)

try:
    while True:
        # Move forward
        move_forward(100)
        time.sleep(5)

        # Brake
        brake_motor()
        time.sleep(2)

        # Turn left
        turn_left(100)
        time.sleep(5)

        # Turn right
        turn_right(100)
        time.sleep(5)

except KeyboardInterrupt:
    GPIO.cleanup()
