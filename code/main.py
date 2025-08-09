import cv2
from detect import detect_objects
from task_holder import assign_processing_label

# Example usage integrating detect_objects
def main():
    known_objects = []
    next_label_index = 1
    distance_threshold = 50

    cap = cv2.VideoCapture(0)

    while True:
        success, frame = cap.read()
        if not success:
            break

        # Perform detection
        current_objects, frame = detect_objects(frame)  # Removed unused variables

        # Assign "processing" label to the object with the smallest numerical suffix
        updated_objects = assign_processing_label(current_objects)
        known_objects = updated_objects

        cv2.imshow("YOLOv8", frame)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC key
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()