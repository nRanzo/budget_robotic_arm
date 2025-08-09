import cv2
import numpy as np
import csv
import os
from datetime import datetime
from ultralytics import YOLO

# Initialize YOLO model once - using relative path to model folder
model_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "model", "yolov8n.pt")
model = YOLO(model_path)

def save_to_csv(frame_objects, filename="detected_objects.csv"):
    """Save detected objects to CSV file with timestamp in csv folder"""
    # Create path to csv folder
    csv_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "csv")
    filepath = os.path.join(csv_dir, filename)
    
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # millisecond precision
    
    # Check if file exists to decide if we need to write headers
    file_exists = os.path.exists(filepath)
    
    with open(filepath, 'a', newline='') as csvfile:
        fieldnames = ['timestamp', 'x1', 'y1', 'x2', 'y2', 'class_name', 'confidence']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
        # Write header if file is new
        if not file_exists:
            writer.writeheader()
        
        # Write each detected object
        for (x1, y1, x2, y2), class_name, confidence in frame_objects:
            writer.writerow({
                'timestamp': timestamp,
                'x1': x1,
                'y1': y1,
                'x2': x2,
                'y2': y2,
                'class_name': class_name,
                'confidence': confidence
            })

def save_current_frame_csv(frame_objects, filename="detection.csv"):
    """Save only current frame objects to CSV file (overwrites previous) in csv folder"""
    # Create path to csv folder
    csv_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "csv")
    filepath = os.path.join(csv_dir, filename)
    
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # millisecond precision
    
    with open(filepath, 'w', newline='') as csvfile:
        fieldnames = ['timestamp', 'x1', 'y1', 'x2', 'y2', 'class_name', 'confidence']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
        # Always write header since we're overwriting
        writer.writeheader()
        
        # Write each detected object from current frame
        for (x1, y1, x2, y2), class_name, confidence in frame_objects:
            writer.writerow({
                'timestamp': timestamp,
                'x1': x1,
                'y1': y1,
                'x2': x2,
                'y2': y2,
                'class_name': class_name,
                'confidence': confidence
            })

def detect_objects(frame):
    current_frame_objects = []
    results = model(frame)

    for r in results:
        for box in r.boxes:
            # point to top left and bottom right are enough to draw boxes
            x1, y1, x2, y2 = box.xyxy[0].int().tolist()
            confidence = box.conf[0].item()
            class_id = int(box.cls[0].item())
            class_name = model.names[class_id]

            label = f"{class_name} {confidence:.2f}"

            # Draw bounding boxes and labels
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            current_frame_objects.append(((x1, y1, x2, y2), class_name, confidence))

    # Save current frame objects to both CSV files
    save_to_csv(current_frame_objects)  # Appends to detected_objects.csv
    save_current_frame_csv(current_frame_objects)  # Overwrites detection.csv
    
    return current_frame_objects, frame

def main():
    cap = cv2.VideoCapture(0)

    while True:
        success, frame = cap.read()
        if not success:
            break

        current_frame_objects, frame = detect_objects(frame)

        cv2.imshow("YOLOv8", frame)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC to exit
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
