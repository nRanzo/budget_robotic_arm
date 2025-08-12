import requests
import time
import csv
import math
import os
from dataclasses import dataclass
from typing import Optional, Tuple

# Constants
ESP32_URL = "http://192.168.4.1/set"
OFFSET_SERVO1 = 110  # corresponds to 180° from camera
NEUTRAL_POSITIONS = [100, 126, 150, 160]

# Configuration
class Config:
    REQUEST_TIMEOUT = 0.3
    RETRY_DELAY = 2.0
    POSITION_TOLERANCE = 2.0
    GOAL_CHANGE_THRESHOLD = 5.0
    MAX_INITIAL_JUMP = 30
    MAX_STEP_SIZE = 2
    MOVEMENT_DELAY = 0.8
    POSITION_CHECK_DELAY = 0.5
    COORDINATE_CHECK_DELAY = 0.5

@dataclass
class Point:
    x: int
    y: int

@dataclass
class PolarCoordinate:
    rho: float
    theta_deg: float

class CSVReader:
    """Handles reading coordinates from CSV files"""
    
    @staticmethod
    def _read_csv_row(file_path: str, columns: dict) -> Optional[dict]:
        """Generic CSV reader that extracts specified columns from first row"""
        try:
            if os.path.exists(file_path):
                with open(file_path, mode="r") as f:
                    reader = csv.DictReader(f)
                    for row in reader:
                        result = {}
                        for key, csv_column in columns.items():
                            result[key] = int(row[csv_column].strip())
                        return result
        except (FileNotFoundError, ValueError, KeyError, OSError) as e:
            print(f"Error reading {file_path}: {e}")
        return None

    @staticmethod
    def read_arm_coordinates() -> Tuple[Optional[int], Optional[int]]:
        """Read arm center coordinates from CSV file"""
        file_path = os.path.join("..", "csv", "arm_coordinates.csv")
        columns = {'x': 'x', 'y': 'y'}
        result = CSVReader._read_csv_row(file_path, columns)
        if result:
            return result['x'], result['y']
        return None, None

    @staticmethod
    def read_current_position() -> Tuple[Optional[int], Optional[int]]:
        """Read current claw position (average of c1 and c2)"""
        file_path = os.path.join("..", "csv", "coordinates_log.csv")
        columns = {'x': 'c1_c2_avg_x', 'y': 'c1_c2_avg_y'}
        result = CSVReader._read_csv_row(file_path, columns)
        if result:
            return result['x'], result['y']
        return None, None

    @staticmethod
    def read_goal_position() -> Tuple[Optional[int], Optional[int]]:
        """Read goal position (c0 coordinates)"""
        file_path = os.path.join("..", "csv", "coordinates_log.csv")
        columns = {'x': 'c0_x', 'y': 'c0_y'}
        result = CSVReader._read_csv_row(file_path, columns)
        if result:
            return result['x'], result['y']
        return None, None

class MathUtils:
    """Mathematical utility functions"""
    
    @staticmethod
    def distance_between_points(point1: Point, point2: Point) -> float:
        """Calculate Euclidean distance between two points"""
        dx = point2.x - point1.x
        dy = point2.y - point1.y
        return math.hypot(dx, dy)
    
    @staticmethod
    def polar_from_points(point1: Point, point2: Point) -> PolarCoordinate:
        """Convert two points to polar coordinates (distance and angle)"""
        dx = point2.x - point1.x
        dy = point2.y - point1.y
        rho = math.hypot(dx, dy)
        # Reflect system on x-axis (negative dy for clockwise rotation)
        theta_deg = (math.degrees(math.atan2(-dy, dx)) + 360) % 360
        return PolarCoordinate(rho, theta_deg)

    @staticmethod
    def angle_diff(goal: float, current: float) -> float:
        """Calculate shortest angular difference between two angles"""
        diff = (goal - current) % 360
        if diff > 180:
            diff -= 360
        return diff

class ServoController:
    """Handles servo communication and control"""
    
    @staticmethod
    def send_command(servo_id: int, angle: float) -> bool:
        """Send servo command with error handling"""
        try:
            response = requests.get(
                ESP32_URL, 
                params={"servo": servo_id, "angle": int(round(angle))}, 
                timeout=Config.REQUEST_TIMEOUT
            )
            print(f"Servo {servo_id} → {angle:.0f}° → Status: {response.status_code}")
            return response.status_code == 200
        except Exception as e:
            print(f"Error with servo {servo_id}: {e}")
            return False

    @staticmethod
    def send_command_with_retry(servo_id: int, angle: float) -> None:
        """Send servo command with automatic retry on failure"""
        while not ServoController.send_command(servo_id, angle):
            print(f"Retrying servo {servo_id} → {angle:.0f}°")
            time.sleep(Config.RETRY_DELAY)

class ClawController:
    """Controls claw opening and closing"""
    
    OPEN_ANGLE = 167
    CLOSE_ANGLE = 140  # Full closed - might use high energy
    
    @classmethod
    def open(cls):
        """Open the claw"""
        ServoController.send_command_with_retry(4, cls.OPEN_ANGLE)
    
    @classmethod
    def close(cls):
        """Close the claw"""
        ServoController.send_command_with_retry(4, cls.CLOSE_ANGLE)

class Joint1Controller:
    """Controls the first joint (rotation)"""
    
    def __init__(self):
        self.last_servo_angle = OFFSET_SERVO1
        self.first_move_done = False
        self.last_goal_angle = None

    def _check_goal_change(self, current_goal_angle: float) -> bool:
        """Check if goal has changed significantly"""
        if (self.last_goal_angle is None or 
            abs(MathUtils.angle_diff(current_goal_angle, self.last_goal_angle)) >= Config.GOAL_CHANGE_THRESHOLD):
            self.first_move_done = False
            self.last_goal_angle = current_goal_angle
            return True
        return False

    def _perform_initial_move(self, delta: float) -> None:
        """Perform initial large movement towards goal"""
        move = delta
        if abs(move) > Config.MAX_INITIAL_JUMP:
            move = Config.MAX_INITIAL_JUMP if move > 0 else -Config.MAX_INITIAL_JUMP
        
        target_angle = self.last_servo_angle + move
        ServoController.send_command_with_retry(1, target_angle)
        self.last_servo_angle = target_angle
        self.first_move_done = True

    def _perform_fine_adjustment(self, delta: float) -> None:
        """Perform small step adjustment"""
        move = (Config.MAX_STEP_SIZE if delta > Config.MAX_STEP_SIZE else 
                -Config.MAX_STEP_SIZE if delta < -Config.MAX_STEP_SIZE else delta)
        
        target_angle = self.last_servo_angle + move
        ServoController.send_command_with_retry(1, target_angle)
        self.last_servo_angle = target_angle

    def move_to_goal(self) -> bool:
        """Move joint 1 to align with goal. Returns True when in position."""
        arm_x, arm_y = CSVReader.read_arm_coordinates()
        current_x, current_y = CSVReader.read_current_position()
        goal_x, goal_y = CSVReader.read_goal_position()

        # Check if all coordinates are available
        if None in (arm_x, arm_y, current_x, current_y, goal_x, goal_y):
            print("Coordinates not available, skipping...")
            return False

        # Convert to points for easier handling
        arm_point = Point(arm_x, arm_y)
        current_point = Point(current_x, current_y)
        goal_point = Point(goal_x, goal_y)

        # Calculate polar coordinates
        current_polar = MathUtils.polar_from_points(arm_point, current_point)
        goal_polar = MathUtils.polar_from_points(arm_point, goal_point)

        print(f"Current - Distance: {current_polar.rho:.2f}, Angle: {current_polar.theta_deg:.2f}°")
        print(f"Goal - Distance: {goal_polar.rho:.2f}, Angle: {goal_polar.theta_deg:.2f}°")

        # Check for significant goal change
        self._check_goal_change(goal_polar.theta_deg)

        # Calculate angular difference
        delta = MathUtils.angle_diff(goal_polar.theta_deg, current_polar.theta_deg)

        # Check if we're close enough
        if abs(delta) <= Config.POSITION_TOLERANCE:
            print("Servo 1 in position")
            return True

        # Perform movement
        if not self.first_move_done:
            self._perform_initial_move(delta)
        else:
            self._perform_fine_adjustment(delta)

        return False

class DistanceAnalyzer:
    """Analyzes distances between key points for joint control
    
    The Two Different Distance Measurements:

    1. current_to_goal - "Distance between current claw and goal"
    This is the direct straight-line distance between:

    Current claw position: (c1_c2_avg_x, c1_c2_avg_y) - where the claw actually is
    Goal position: (c0_x, c0_y) - where we want to pick up the object
    This tells us how far apart the claw and target are in the camera view.

    2. reach_difference - "Reach difference (goal - current)"
    This is the difference in reach distance from the arm base:

    arm_to_current: Distance from arm base to current claw position
    arm_to_goal: Distance from arm base to goal position
    reach_difference = arm_to_goal - arm_to_current
    This tells us if the arm needs to be extended or retracted

    Example:
        Arm Base (1169, 776)
            |
            |<-- arm_to_current = 200px
            |
        Current Claw (avg c1,c2)
            |
            |<-- current_to_goal = 50px
            |
        Goal (c0) 
            |
            |<-- arm_to_goal = 250px

        current_to_goal = 50px (direct distance between claw and target)
        reach_difference = 250 - 200 = 50px (need to extend arm by 50px worth of reach)
    """
    
    @staticmethod
    def get_all_distances() -> Optional[dict]:
        """Get all relevant distances for arm control"""
        current_x, current_y = CSVReader.read_current_position()
        goal_x, goal_y = CSVReader.read_goal_position()
        arm_x, arm_y = CSVReader.read_arm_coordinates()
        
        if None in (current_x, current_y, goal_x, goal_y, arm_x, arm_y):
            return None
        
        current_point = Point(current_x, current_y)
        goal_point = Point(goal_x, goal_y)
        arm_point = Point(arm_x, arm_y)
        
        return {
            'current_to_goal': MathUtils.distance_between_points(current_point, goal_point),
            'arm_to_current': MathUtils.distance_between_points(arm_point, current_point),
            'arm_to_goal': MathUtils.distance_between_points(arm_point, goal_point),
            'goal_closer_than_current': MathUtils.distance_between_points(arm_point, goal_point) < MathUtils.distance_between_points(arm_point, current_point)
        }
    
    @staticmethod
    def get_reach_difference() -> Optional[float]:
        """Get the difference in reach between current position and goal"""
        distances = DistanceAnalyzer.get_all_distances()
        if distances is None:
            return None
        return distances['arm_to_goal'] - distances['arm_to_current']

class ArmController: # basically joint2_3 and all the rest above
    """Main arm controller orchestrating all movements"""
    
    def __init__(self):
        self.joint1_controller = Joint1Controller()
        self.claw_controller = ClawController()
        # Track current servo positions for joints 2 and 3
        self.servo2_angle = 130  # Starting retracted position
        self.servo3_angle = 140  # Starting retracted position

    def joint2_3_retract(self):
        """Instantly retract arm to starting position"""
        print("Retracting arm to starting position...")
        self.servo2_angle = 130
        self.servo3_angle = 140
        
        ServoController.send_command_with_retry(3, -self.servo3_angle)
        time.sleep(0.5)
        ServoController.send_command_with_retry(2, self.servo2_angle)
        print(f"Arm retracted: Servo 2 → {self.servo2_angle}°, Servo 3 → {self.servo3_angle}°")

    def joint2_3_extend(self):
        """Incrementally extend arm toward goal"""
        print("Extending arm toward goal...")
        
        # Distance threshold - when we're this close, do one final step and stop
        CLOSE_ENOUGH_THRESHOLD = 0.0  # pixels
        ANGLE_INCREMENT = 5  # degrees
        STEP_DELAY = 1.0  # seconds between steps
        MAX_ANGLE = 180  # Maximum safe angle for servos
        
        while True:
            # Get current distances
            distances = DistanceAnalyzer.get_all_distances()
            if distances is None:
                print("Cannot get distances - coordinates not available")
                break
            
            current_to_goal_distance = distances['current_to_goal']
            reach_difference = DistanceAnalyzer.get_reach_difference()
            
            print(f"Distance to goal: {current_to_goal_distance:.1f} pixels")
            print(f"Reach difference: {reach_difference:.1f} pixels")
            print(f"Current angles: Servo 2 = {self.servo2_angle}°, Servo 3 = {self.servo3_angle}°")
            
            # Check if we're close enough to goal
            if current_to_goal_distance <= CLOSE_ENOUGH_THRESHOLD:
                print(f"Close enough to goal ({current_to_goal_distance:.1f} <= {CLOSE_ENOUGH_THRESHOLD})")
                print("Doing one final extension step...")
                
                # Do one final step and exit
                if self.servo2_angle < MAX_ANGLE and self.servo3_angle < MAX_ANGLE:
                    self.servo2_angle += ANGLE_INCREMENT
                    self.servo3_angle += ANGLE_INCREMENT
                    
                    ServoController.send_command_with_retry(2, self.servo2_angle)
                    time.sleep(0.5)
                    ServoController.send_command_with_retry(3, self.servo3_angle)
                    print(f"Final step: Servo 2 → {self.servo2_angle}°, Servo 3 → {self.servo3_angle}°")
                
                print("Extension complete - positioned above goal")
                break
            
            # Check if we need to extend (goal is farther from arm base than current position)
            if reach_difference > 0:
                print("Need to extend further...")
                
                # Check if we can still extend safely
                if self.servo2_angle >= MAX_ANGLE or self.servo3_angle >= MAX_ANGLE:
                    print("Reached maximum extension - cannot extend further")
                    break
                
                # Extend both servos by the same amount
                self.servo2_angle += ANGLE_INCREMENT
                self.servo3_angle += ANGLE_INCREMENT
                
                # Clamp to maximum angles
                self.servo2_angle = min(self.servo2_angle, MAX_ANGLE)
                self.servo3_angle = min(self.servo3_angle, MAX_ANGLE)
                
                ServoController.send_command_with_retry(2, self.servo2_angle)
                time.sleep(0.5)
                ServoController.send_command_with_retry(3, self.servo3_angle)
                print(f"Extended: Servo 2 → {self.servo2_angle}°, Servo 3 → {self.servo3_angle}°")
                
            else:
                print("Goal is closer than current position - no need to extend further")
                break
            
            # Wait before next step
            time.sleep(STEP_DELAY)

    def joint2_3_lower(self):
        """Lower the claw by adjusting servo 3 and extend servo 2 a bit more"""
        print("Lowering claw and extending arm slightly...")
        LOWER_AMOUNT = 35  # degrees to lower servo 3
        EXTEND_AMOUNT = 35  # degrees to extend servo 2 more
        
        # Extend servo 2 a bit more for better reach
        self.servo2_angle += EXTEND_AMOUNT
        self.servo2_angle = min(self.servo2_angle, 180)  # Don't exceed 180°
        
        # Lower servo 3 to drop the claw down
        self.servo3_angle -= LOWER_AMOUNT
        self.servo3_angle = max(self.servo3_angle, 90)  # Don't go below 90°
        
        ServoController.send_command_with_retry(2, self.servo2_angle)
        time.sleep(0.5)
        ServoController.send_command_with_retry(3, self.servo3_angle)
        print(f"Arm extended and claw lowered: Servo 2 → {self.servo2_angle}°, Servo 3 → {self.servo3_angle}°")

    def joint2_3_raise(self):
        """Raise the claw back to horizontal"""
        print("Raising claw...")
        
        # Restore servo 3 to its extended position (same as servo 2)
        self.servo3_angle = self.servo2_angle
        
        ServoController.send_command_with_retry(3, self.servo3_angle)
        print(f"Claw raised: Servo 3 → {self.servo3_angle}°")

    def execute_pick_sequence(self):
        """Execute complete pick and place sequence"""
        while True:
            print("=== Starting pick sequence ===")
            
            # Start with retracted position
            self.joint2_3_retract()
            time.sleep(2)
            
            # Align joint 1
            print("Aligning joint 1...")
            while not self.joint1_controller.move_to_goal():
                time.sleep(Config.MOVEMENT_DELAY)

            # Open claw
            print("Opening claw...")
            self.claw_controller.open()
            time.sleep(2)
            
            # Extend arm toward goal
            print("Extending arm toward goal...")
            self.joint2_3_extend()
            time.sleep(2)
            
            # Lower claw to object
            print("Lowering claw to object...")
            self.joint2_3_lower()
            time.sleep(2)
            
            # Close claw
            print("Closing claw...")
            self.claw_controller.close()
            time.sleep(2)

            # Raise claw
            print("Raising claw...")
            self.joint2_3_raise()
            time.sleep(2)

            # Retract arm
            print("Retracting arm...")
            self.joint2_3_retract()
            time.sleep(2)
            
            # TODO: Add bin positioning logic here
            
            print("=== Pick sequence completed ===")
            time.sleep(10)

def main():
    """Main execution function"""
    print("Starting arm movement control...")
    arm_controller = ArmController()
    arm_controller.execute_pick_sequence()

if __name__ == "__main__":
    main()