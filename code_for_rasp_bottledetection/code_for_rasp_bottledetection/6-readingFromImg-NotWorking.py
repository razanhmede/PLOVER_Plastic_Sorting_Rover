import cv2
from ultralytics import YOLO

# the parent folder of everything here is named PLASTIC-DETECTION
# When you want to reference something, eventhough I'm in the Tutorial folder, it automatically goes 
# to the parent folder, PLASTIC-DETECTION. So if you are referencing a file or a folder, you should 
# reference it as if you are in the parent folder.

model = YOLO("./Weights/yolov5n.pt")
results = model("img0.JPG",show=True)
cv2.waitKey(0)