def assign_processing_label(detected_objects):
    """
    detected_objects should be a list of ((x1, y1, x2, y2), class_name, confidence).
    This finds the object with the highest confidence and changes that object's label to "processing".
    Only ONE object should be labeled "processing" at a time.
    """
    max_confidence = -1
    max_index = None
    previous_processing_index = None

    # Find the object with the highest confidence
    for i, (bbox_data, class_name, confidence) in enumerate(detected_objects):
        if class_name != "processing" and confidence > max_confidence:
            max_confidence = confidence
            max_index = i
        elif class_name == "processing":
            previous_processing_index = i

    # Reset any existing "processing" labels to their original class name
    if previous_processing_index is not None:
        bbox_data, _, confidence = detected_objects[previous_processing_index]
        detected_objects[previous_processing_index] = (bbox_data, "object", confidence)  # Revert to "object" or a more appropriate default

    # Assign "processing" label to the object with the highest confidence
    if max_index is not None:
        bbox_data, class_name, confidence = detected_objects[max_index]
        detected_objects[max_index] = (bbox_data, "processing", confidence)

    return detected_objects

if __name__ == "__main__":
    # Example usage
    objects = [(((100, 200, 150, 250)), "object", 0.8), (((150, 250, 200, 300)), "object", 0.9), (((123, 456, 150, 500)), "object", 0.7), (((50, 50, 75, 75)), "processing", 0.95)]
    print("Before:", objects)
    updated = assign_processing_label(objects)
    print("After:", updated)