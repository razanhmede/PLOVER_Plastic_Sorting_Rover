import RPi.GPIO as GPIO
import time

# Define the GPIO pins connected to the ultrasonic sensor
TRIG_PIN = 6
ECHO_PIN = 5

# Initialize GPIO settings
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

def measure_distance():
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, GPIO.LOW)

    pulse_start = time.time()
    pulse_end = time.time()

    while GPIO.input(ECHO_PIN) == GPIO.LOW:
        pulse_start = time.time()
        if pulse_start - pulse_end > 0.1:  # Timeout if echo signal not received within 0.1 seconds
            print("Timeout: Echo signal not received")
            return None

    while GPIO.input(ECHO_PIN) == GPIO.HIGH:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance_cm = pulse_duration * 17150
    distance_cm = round(distance_cm, 2)

    return distance_cm

try:
    while True:
        distance = measure_distance()
        print("Distance:", distance, "cm")
        time.sleep(1)  # Wait for 1 second before the next measurement

except KeyboardInterrupt:
    GPIO.cleanup()
