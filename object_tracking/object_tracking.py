import cv2
from ultralytics import YOLO

model = YOLO("yolov10n.pt")

video_path = "/Users/shreyessridhara/Documents/boatvideo.mp4"
cap = cv2.VideoCapture(video_path)

known_height = 5.0  # Example: 5 meters

# Camera focal length (in pixels), obtained from camera calibration
focal_length = 1000  # Example value

ret = True
while ret:
    ret, frame = cap.read()
    
    if ret:
        results = model.track(frame, persist=True)
        frame_ = results[0].plot()
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                # Extract the bounding box coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # Calculate the perceived height of the boat in the image
                perceived_height = y2 - y1
                
                # Calculate the distance to the boat
                distance = (known_height * focal_length) / perceived_height
                
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
    
    
    
    
    