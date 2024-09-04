# PLOVER_Plastic_Sorting_Rover

## Project Overview

Our project, the Plastic Sorting Rover, integrates a camera, rover, gripper, and ultrasonic sensor connected to a Raspberry Pi on a rover platform. Using a YOLOv8 deep learning model, it detects plastic bottles and navigates towards them by aligning with the bottle's position relative to the frame center. Upon reaching close proximity, inverse kinematics is integrated to grip the bottle accurately. After gripping, it rotates to position the gripper towards a cardboard receptacle at the back, releases the bottle, and returns to its initial position for continued scanning, offering efficient plastic bottle sorting capabilities. 

## Components List 

•	Rover 
•	Ultrasonic Sensor
•	Raspberry pi 3 B+
•	Raspberry pi Camera
•	2 MG966R servo motors
•	2 SG90 servo motors
•	Power Bank
•	Adapter
•	3D printed parts

## Summary of the functionality steps

•	Object Detection: The camera onboard the rover sends a streamed video using a Flask server to the YOLOv8 model on the laptop to detect plastic bottles within its field of view.

•	Navigation: Upon detecting a bottle, the rover calculates the x-position of the bottle with respect to the center of the frame. It then initiates navigation by turning left or right until the bottle is aligned with the center.

•	Approach and Gripping: As the rover approaches the bottle, the ultrasonic sensor reads the distance. Once the distance reaches 15 cm, indicating proximity, the rover halts its movement.

•	Gripper Control: The gripper mechanism utilizes inverse kinematics to calculate servo positions based on the x-distance from the ultrasonic sensor and the y-coordinate of the bottle center relative to the frame. This allows precise gripping of the bottle.

•	Sorting and Placement: After gripping the bottle, the rover rotates 180 degrees to position the gripper towards a cardboard box located at the back. The gripper then releases the bottle into the box.

•	Return to Initial Position: Once the bottle is sorted, the rover returns to its initial position to resume scanning for additional objects.
