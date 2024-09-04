from picamera2 import Picamera2

picam2 = Picamera2()

picam2.start_and_capture_file("test926.jpg")
# picam2.start_and_record_video("test222.mp4",duration=10, show_preview=True)
