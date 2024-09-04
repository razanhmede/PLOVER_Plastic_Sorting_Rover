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
    # Call the function with updated coordinates
    process_coordinates(x_center, y_center)
    return 'Coordinates updated successfully!'

def process_coordinates(x_center, y_center):
    # Perform desired actions based on x_center and y_center values
    # For example, print them
    print(f'Processing coordinates - x_center: {x_center}, y_center: {y_center}')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)
