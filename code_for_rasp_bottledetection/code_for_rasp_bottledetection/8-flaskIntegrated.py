from flask import Flask, Response, request
from picamera2 import Picamera2
import cv2
import numpy as np

app = Flask(__name__)
picam2 = Picamera2()

x_center = 0
y_center = 0

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
    # now I have x_center and y_center delayed 15s from real time
    robot()
    # measure_distance()
    # Call the function with updated coordinates
    process_coordinates(x_center, y_center)
    return 'Coordinates updated successfully!'

def process_coordinates(x_center, y_center):
    # Perform desired actions based on x_center and y_center values
    # For example, print them
    print(f'Processing coordinates - x_center: {x_center}, y_center: {y_center}')

##################################################################
# Robot code
import RPi.GPIO as GPIO
import time
import cv2
import math

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
# servo1_pin = 13  # GPIO13 for servo1 (joint 1)
# servo2_pin = 19  # GPIO19 for servo2 (joint 2)
# gripper_pin = 26 # GPIO26 for gripper
# base_pin = 21    # GPIO21 for base rotation

# Constants for arm lengths
# L1 = 12  # Length of the first arm segment
# L2 = 17  # Length of the second arm segment

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
# GPIO.setup(servo1_pin, GPIO.OUT)
# GPIO.setup(servo2_pin, GPIO.OUT)
# GPIO.setup(gripper_pin, GPIO.OUT)
# GPIO.setup(base_pin, GPIO.OUT)

# Initialize PWM for servo motors
# servo1 = GPIO.PWM(servo1_pin, 50)  # 50 Hz frequency
# servo2 = GPIO.PWM(servo2_pin, 50)
# gripper = GPIO.PWM(gripper_pin, 50)
# base = GPIO.PWM(base_pin, 50)

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

# Function to open the gripper
# def open_gripper():
#     gripper.start(0)  # Start PWM with duty cycle 0 for fully open
#     time.sleep(1)     # Wait for gripper to open
#     print("Gripper is open.")

# # Function to close the gripper
# def close_gripper():
#     gripper.start(100)  # Start PWM with duty cycle 100 for fully closed
#     time.sleep(1)       # Wait for gripper to close
#     print("Gripper is closed.")

# # Function to rotate the base
# def rotate_base():
#     base.start(7.5)  # Start PWM with duty cycle corresponding to 0 degrees
#     time.sleep(1)    # Wait for base rotation to complete
#     print("Base rotated to place object.")
#     open_gripper()   # Open gripper to release the object
#     print("Object placed. Gripper is open.")
#     base.start(2.5)  # Rotate back to initial position
#     time.sleep(1)
#     print("Base returned to initial position.")

# # Function to calculate inverse kinematics
# def inv_kinematics(x, y):
#     # Compute distance from origin to point
#     d = math.sqrt(x*2 + y*2)
#     print("Distance from origin to point:", d)

#     # Compute angle to point (in radians) using atan2 for full quadrant coverage
#     theta = math.atan2(y, x)
#     print("Theta:", math.degrees(theta))

#     # Check if the point is within reach
#     if (L1 + L2) >= d and abs(L1 - L2) <= d:
#         # Calculate joint angles using the cosine law
#         beta = math.acos((L1*2 + L22 - d*2) / (2 * L1 * L2))
#         alpha = math.acos((L1*2 + d2 - L2*2) / (2 * L1 * d))

#         theta1 = math.degrees(theta + alpha)
#         theta2 = 180 - math.degrees(beta - alpha)

#         servo1.start(2.5 + (theta1 / 18))  # Convert degrees to PWM duty cycle
#         servo2.start(2.5 + (theta2 / 18))
#         print("Arm moved to position.")
#     else:
#         print("Position out of reach.")
#         initial_position()

# # Function to set the arm to initial position
# def initial_position():
#     servo1.start(7.5)  # Set servo to middle position
#     servo2.start(7.5)
#     base.start(7.5)    # Set base to initial position
#     open_gripper()     # Open the gripper in the initial position

# PWM setup for motor speed control
pwm_frequency = 1000
pwm_a = GPIO.PWM(ENA, pwm_frequency)
pwm_b = GPIO.PWM(ENB, pwm_frequency)

# def ultrasonicCalibration(difference):
#     if difference > 0:
#         # object aa yamin l nss
#         # rouh aal yamin
#         turn_right(100)
#     elif difference < 0:
#         turn_left(100)
#     else:
#         # return to robot function
        
#     # baad ma ballash awal direction
#     # 1 ultrasonic yred value w ballesh timer
#     # 2 bas ta ybatel yred value, wa2ef timer w brake motor
#     # 3 bas ta 3rft l timer, turn in opposite direction for timer/2 then brake_motor() then return
    
    

def robot():
    try:
        # initial_position()
        while True:
            # Measure distance
            dist = measure_distance()
            print("Distance: {} cm".format(dist))

            # Insert code to get x and y center of the object
            object_center_x = x_center
            object_center_y = y_center

            # Calculate difference between object center and frame center
            frame_center_x = 320  # Assuming the frame width is 640
            diff = object_center_x - frame_center_x

            if diff < -5 or diff >5:
            # Adjust rover movement based on the difference
                if diff < -5:
                    turn_left(100)  # Turn left at a reduced speed
                    time.sleep(0.5)
                    brake_motor()
                elif diff > 5:
                    turn_right(100)  # Turn right at a reduced speed
                    time.sleep(0.5)
                    brake_motor()
            
            move_forward(100)  # Move forward at a slow speed
            
            time.sleep(5)
            # Check if the distance to the object is less than or equal to 15 cm
            if dist <= 15:
                brake_motor()  # Stop the rover
                # inv_kinematics(object_center_x, object_center_y)
                # time.sleep(2)
                # close_gripper()
                # time.sleep(2)
                # rotate_base()
                # time.sleep(2)
                # open_gripper()
                # time.sleep(2)
                # initial_position()

            time.sleep(0.1)

    except KeyboardInterrupt:
        GPIO.cleanup()

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)
