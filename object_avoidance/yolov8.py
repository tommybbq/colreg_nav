from ultralytics import YOLO
from ultralytics.models.yolo.detect import DetectionPredictor
import cv2

model = YOLO("yolov8s.pt")
results = model.predict(source="/Users/shreyessridhara/Documents/yellowred.mov", show=True) #essentially just use the 360-degree camera as source!
print(results)