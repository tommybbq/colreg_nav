import cv2
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort  # Assuming DeepSort implementation

# Initialize YOLOv8 model
model = YOLO("yolov8n.pt")

# Initialize DeepSort tracker
deepsort = DeepSort()

video_path = "/Users/shreyessridhara/Documents/boatvideo.mp4"
cap = cv2.VideoCapture(video_path)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    # Perform object detection with YOLOv8
    results = model(frame)
    
    # Extract detections from YOLOv8 results
    detections = []
    
    for obj in results.pred[0]:
        bbox = [
            int(obj['xmin']),
            int(obj['ymin']),
            int(obj['xmax']),
            int(obj['ymax'])
        ]
        confidence = float(obj['confidence'])
        class_name = obj['name']
        
        detections.append((bbox, confidence, class_name))
    
    # Update tracks with DeepSort
    tracks = deepsort.update_tracks(detections, frame=frame)
    
    # Visualize tracked objects
    for track in tracks:
        bbox = track.to_tlbr()  # Get the bounding box in format (top, left, bottom, right)
        class_name = track.get_class()  # Get the class name
        cv2.rectangle(frame, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (0, 255, 0), 2)
        cv2.putText(frame, f"{class_name} {track.track_id}", (int(bbox[0]), int(bbox[1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # Display the resulting frame
    cv2.imshow('frame', frame)
    
    # Exit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()