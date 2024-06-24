from ultralytics import YOLO
from ultralytics.models.yolo.detect import DetectionPredictor
import cv2

model = YOLO("yolov8s.pt") #/Users/shreyessridhara/Documents/colreg_nav/object_avoidance/runs/detect/train2/weights/bestbuoys.pt
results = model.predict(source="/Users/shreyessridhara/Documents/boatvideo.mp4", show=True, conf=0.4) #essentially just use the 360-degree camera as source!
print(results)

'''from ultralytics import YOLO
import cv2

# Load the YOLOv8 model
model = YOLO("yolov8s.pt")

# Define the target class you want to detect
target_class = 'boat'  # Replace with the class name you are interested in

# Open the video file
video_path = "/Users/shreyessridhara/Documents/yellowred.mov"
cap = cv2.VideoCapture(video_path)

# Process each frame in the video
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    # Perform prediction
    results = model.predict(frame)
    
    # Check the type and structure of results
    if isinstance(results, list):
        # If results is a list, iterate over each prediction
        for pred in results:
            class_name = model.names[int(pred[5])]
            confidence = pred[4]
            
            # Print detection info
            print(f"Detected {class_name} with confidence {confidence:.2f}")
            
            # Additional statement for specific class (e.g., 'boat')
            if class_name == target_class:
                print("Oh, it's a boat!")
    
    elif isinstance(results, dict):
        # If results is a dictionary, access predictions accordingly
        detections = results['predictions']  # Adjust key based on your model's output structure
        for detection in detections:
            class_name = model.names[int(detection[5])]
            confidence = detection[4]
            
            # Print detection info
            print(f"Detected {class_name} with confidence {confidence:.2f}")
            
            # Additional statement for specific class (e.g., 'boat')
            if class_name == target_class:
                print("Oh, it's a boat!")
    
    # Display the frame (optional)
    cv2.imshow('YOLOv8 Detection', frame)
    
    # Exit loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release video capture and close all windows
cap.release()
cv2.destroyAllWindows()
'''