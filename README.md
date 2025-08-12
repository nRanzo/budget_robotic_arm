# Autonomous Low-Cost Robotic Arm with Computer Vision

Learn more at [this Shimeji page](https://shimeji.vercel.app/armrobot).

## Introduction

This open-source project explores autonomous object detection and manipulation using a budget-friendly robotic arm (approx. €50–100 for all components).  
The system is powered by an **ESP32-S3** microcontroller and **PCA9685** servo driver, with four **SG90** servomotors mounted on an acrylic frame.

### Computer running Python scripts

Two vision approaches were tested:

- **YOLOv8-based detection** – high-accuracy object recognition, but with performance and stability limitations in low-cost setups.
- **Light segmentation** – reduces computation by up to 80%, offering robust performance under controlled lighting conditions.

The control logic uses a **kinematic model** with iterative feedback correction to compensate for servo inaccuracies and mechanical play, enabling successful object acquisition despite hardware constraints.

All components, code, and instructions are provided to ensure reproducibility.  
Potential improvements include feedback actuators, dynamic calibration, advanced pathfinding algorithms (DFS/A*), and refined mechanical design for higher precision.

---

## ESP32 Setup

The ESP32 is connected to SG90 servos and works as a Wi-Fi access point.  
The computer can connect to this network and send **GET** requests with parameters specifying the servo number and desired new angle.

- ESP32 firmware code is located in the `esp32/` folder.
- A `demo.py` script is provided to repeatedly send commands, useful for checking if the system works correctly before running the full pipeline.

---

## How to Run – Light Segmentation (**Recommended**)

Two scripts must run simultaneously:

1. `detect_light.py` – Implements light segmentation logic and updates the `.csv` files.  
2. `arm_movs_refactored.py` – Reads CSV values and moves the robot accordingly.

---

## How to Run – YOLOv8

Two scripts must run simultaneously:

1. `detect.py` – Implements the YOLOv8 object detection logic *(updates `detected_object.csv`)*.  
2. *(Currently, there is a missing processing step that should convert `detected_object.csv` into both `arm_coordinates.csv` and `coordinates_log.csv`.)*  
3. `arm_movs_refactored.py` – Reads CSV values and moves the robot accordingly.
---

## Notes

- Tests were primarily conducted using **light segmentation / binary mask** logic, which provided the most consistent results.
- YOLOv8 was functional but less stable on low-cost hardware without optimizations.
- **YOLOv8 model files are not included**. Please download them from the official YOLOv8 repository on GitHub and place them in the `model/` folder if required.
- **CSV files are provided** as examples to illustrate the expected format and values. They will be constantly overwritten as soon as you run the scripts.

---