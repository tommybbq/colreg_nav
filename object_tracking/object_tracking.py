from ultralytics import YOLO
import cv2

model = YOLO("yolov8n.pt")

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
    
    
    
    
    
    