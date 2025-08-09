import cv2
import numpy as np
import csv
import os

def find_coordinates(binary, n=3):
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    centroids = []
    for cnt in contours[:n]:
        M = cv2.moments(cnt)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            centroids.append((cx, cy))
    return centroids

def read_arm_coordinates():
    """Read arm center coordinates from CSV file"""
    arm_csv_file = os.path.join("..", "csv", "arm_coordinates.csv")
    arm_x, arm_y = None, None
    
    try:
        if os.path.exists(arm_csv_file):
            with open(arm_csv_file, mode="r") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    arm_x = int(row['x'].strip())
                    arm_y = int(row['y'].strip())
                    break  # Only read the first row
    except (FileNotFoundError, ValueError, KeyError):
        pass  # If file doesn't exist or has issues, arm coordinates remain None
    
    return arm_x, arm_y

def segment_by_brightness(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    kernel = np.ones((10, 10), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        if cv2.contourArea(cnt) > 500:
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    return binary, frame

def main():
    cap = cv2.VideoCapture(0)

    # Updated CSV path to save in csv folder
    csv_file = os.path.join("..", "csv", "coordinates_log.csv")
    
    # Create csv directory if it doesn't exist
    os.makedirs(os.path.dirname(csv_file), exist_ok=True)
    
    frame_count = 0
    while True:
        success, frame = cap.read()
        if not success:
            break

        binary, result = segment_by_brightness(frame)
        centers = find_coordinates(binary)

        # Read arm coordinates
        arm_x, arm_y = read_arm_coordinates()

        avg_x, avg_y = 0, 0  # Initialize with 0 instead of None
        if len(centers) >= 3:
            x1, y1 = centers[1]
            x2, y2 = centers[2]
            avg_x = (x1 + x2) // 2
            avg_y = (y1 + y2) // 2
            cv2.circle(result, (avg_x, avg_y), 6, (0, 0, 255), -1)
            cv2.putText(result, "avg", (avg_x + 5, avg_y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # Disegna i centroidi
        for i, (x, y) in enumerate(centers):
            cv2.circle(result, (x, y), 5, (255, 0, 0), -1)
            cv2.putText(result, f"{i}", (x + 5, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        # Draw arm coordinates if available
        if arm_x is not None and arm_y is not None:
            cv2.circle(result, (arm_x, arm_y), 8, (255, 255, 0), -1)  # Cyan circle
            cv2.putText(result, "arm", (arm_x + 10, arm_y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

        # Overwrite CSV file each time - move file opening inside the loop
        with open(csv_file, mode="w", newline="") as f:
            writer = csv.writer(f)
            # Write header
            writer.writerow(["frame", "c0_x", "c0_y", "c1_x", "c1_y", "c2_x", "c2_y", "c1_c2_avg_x", "c1_c2_avg_y"])
            
            # Write current frame data with zeros instead of None/empty values
            c0 = centers[0] if len(centers) > 0 else (0, 0)
            c1 = centers[1] if len(centers) > 1 else (0, 0)
            c2 = centers[2] if len(centers) > 2 else (0, 0)
            writer.writerow([frame_count, c0[0], c0[1], c1[0], c1[1], c2[0], c2[1], avg_x, avg_y])

        # Mostra
        cv2.imshow("Binary Mask", binary)
        cv2.imshow("Segmented", result)

        frame_count += 1
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()