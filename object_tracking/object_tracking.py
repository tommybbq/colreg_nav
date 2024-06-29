import cv2
from ultralytics import YOLO

# Initialize the YOLO model with the path to your best.pt file
model = YOLO("/Users/shreyessridhara/Documents/colreg_nav/object_tracking/best.pt")
#https://universe.roboflow.com/cheng-nvnun/cheng-scgtz/browse?queryText=class%3Afish-b&pageSize=50&startingIndex=0&browseQuery=true

# Path to your video file
video_path = "/Users/shreyessridhara/Documents/container.mp4"
cap = cv2.VideoCapture(video_path)

# Dictionary to store average lengths of different boat classes (in meters)
known_lengths = {
    'container': 300.0,
    'cruise': 250.0,
    'fish-b': 5.0, #15
    'sail boat': 5.0,#14
    'warship': 150.0
}

# Camera focal length (in pixels), obtained from camera calibration
focal_length = 2000  # Example value

ret = True
while ret:
    ret, frame = cap.read()
    
    if ret:
        results = model.track(frame, persist=True)
        frame_ = results[0].plot()
        
        for result in results:
            boxes = result.boxes
            classes = result.names
            for box, cls in zip(boxes, classes):
                # Extract the bounding box coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # Calculate the perceived length of the boat in the image
                perceived_length = x2 - x1
                
                # Get the known length for the detected boat class
                known_length = known_lengths.get(cls, 10.0)  # Default to 10.0 meters if class not found
                
                # Calculate the distance to the boat
                distance = (known_length * focal_length) / perceived_length
                
                # Display the distance on the frame in black, offset to avoid overlapping
                cv2.putText(frame_, f'Distance: {distance:.2f}m', (x1, y1 - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
        
        cv2.imshow("frame", frame_)
        if cv2.waitKey(25) & 0xFF == ord("q"):
            break

cap.release()
cv2.destroyAllWindows()



'''from ultralytics import YOLO
import cv2

model = YOLO("yolov10n.pt")

video_path = "/Users/shreyessridhara/Documents/boatvideo.mp4"

cap = cv2.VideoCapture(video_path)

ret = True
while ret:
    ret, frame = cap.read()
    
    if ret:
        results = model.track(frame, persist=True)
        frame_ = results[0].plot()
        
        cv2.imshow("frame", frame_)
        if cv2.waitKey(25) & 0xFF == ord("q"):
            break
    '''
    
    
    
    
    