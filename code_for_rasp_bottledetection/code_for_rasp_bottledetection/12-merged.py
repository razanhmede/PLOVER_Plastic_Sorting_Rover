from flask import Flask, Response, request
from picamera2 import Picamera2
import cv2
import numpy as np

app = Flask(__name__)
picam2 = Picamera2()

x_center = 0
y_center = 0
dist= 0

@app.route('/')
def index():
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

def gen():
    picam2.start()
    while True:
        frame = picam2.capture_array()
        
        # Convert the frame to RGB color space
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        ret, jpeg = cv2.imencode('.jpg', frame_rgb)
        if not ret:
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n\r\n')

# Route to handle POST request from laptop
@app.route('/update_coordinates', methods=['POST'])
def update_coordinates():
    global x_center, y_center
    data = request.json
    x_center = data['x_center']
    y_center = data['y_center']
    # Call the function with updated coordinates
    print("weslo l coordinates aal pi", x_center, y_center)
    # robot()
    print("Khelset l robot method")
    return 'Coordinates updated successfully!'


##################################################################

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

theta1=0
theta2=0

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

# Function to measure distance using ultrasonic sensor
def measure_distance():
    GPIO.output(TRIG_PIN, GPIO.LOW)
    time.sleep(0.5)
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, GPIO.LOW)

    pulse_start = time.time()
    pulse_end = time.time()

    while GPIO.input(ECHO_PIN) == GPIO.LOW:
        pulse_start = time.time()

    while GPIO.input(ECHO_PIN) == GPIO.HIGH:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance_cm = pulse_duration * 17150
    distance_cm = round(distance_cm, 2)
    return distance_cm

# PWM setup for motor speed control
pwm_frequency = 1000
pwm_a = GPIO.PWM(ENA, pwm_frequency)
pwm_b = GPIO.PWM(ENB, pwm_frequency)


def rotate_base():
    
    step_size = 0.1  # Adjust as needed
    delay = 0.05     # Adjust as needed
        
    current_duty_cycle1 = 2.5
    target_duty_cycle1 = 12.5
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

        theta1 = 180-(theta + alpha +40 )
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
        #initial_position()
        # GPIO.cleanup()
        # exit()
    else:
        print("Position out of reach.")
        initial_position()
 
def initial_position():
    servo1.ChangeDutyCycle(7.5)  # Center position for servo1
    servo2.ChangeDutyCycle(2.5)  # Center position for servo2
    base.ChangeDutyCycle(2.5)     # Center position for base
    open_gripper()

# Function to open gripper
def open_gripper():
    gripper.ChangeDutyCycle(12)  # Open gripper
    sleep(1)
    print("Gripper is open.")

# Function to close gripper
def close_gripper():
    gripper.ChangeDutyCycle(2.5)  # Close gripper
    sleep(1)
    print("Gripper is closed.")
           
# Main program
def robot():
    #initial_position()
    try:
        
        dist = measure_distance()
        print("Distance: {} cm".format(dist))
        #insert code to get x and y center of the object
        object_center_x= x_center
        object_center_y= y_center


        # Calculate difference between object center and frame center
        frame_center_x = 320  # Assuming the frame width is 640
        diff = object_center_x - frame_center_x
        print("Difference: ", diff)

        # Adjust rover movement based on the difference
        if diff < -100:
            print("turning left")
            turn_left(100)  # Turn left at a reduced speed
            time.sleep(0.5)
            brake_motor()
            print("stopped turning left")
            

        elif diff > 100:
            print("turning right")
            turn_right(100)  # Turn left at a reduced speed
            time.sleep(0.5)
            brake_motor()
            print("stopped turning right")
            

        else:
            
            move_forward(100)  # Move forward at a slow speed
            #time.sleep(1)
            while dist>15:
                print("ballash yemshe la eddem")
                # Measure distance
                dist = measure_distance()
                print("Distance: {} cm".format(dist))
                
        
            #Check if the distance to the object is less than or equal to 15 cm
            
            print("entered distance less than 15")
            brake_motor()  # Stop the rover
            time.sleep(5)
            # ballash sheghel l gripper
            initial_position()
            time.sleep(5)
            inv_kinematics(dist, 12)


        
     
    except KeyboardInterrupt:
        GPIO.cleanup()

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)
