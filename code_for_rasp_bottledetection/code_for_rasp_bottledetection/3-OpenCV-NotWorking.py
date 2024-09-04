import cv2
import time
print(cv2.__version__)

def main():
    # Open the camera
    cap = cv2.VideoCapture(0)
    # cap = cv2.VideoCapture('/dev/video0')
    # cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

    # Check if camera opened successfully
    if not cap.isOpened():
        print("Error: Failed to open camera")
        return

    # Set the camera resolution to 640x480
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Read and display frames from the camera
    while True:
        time.sleep(6)
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame")
            break

        # Display the frame
        cv2.imshow('Frame', frame)

        # Exit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
