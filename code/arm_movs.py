import requests
import time
import csv
import math
import os

ESP32_URL = "http://192.168.4.1/set"
OFFSET_SERVO1 = 110  # corrisponde a 180° dalla camera
neutral = [100, 126, 150, 160]


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

def read_current_position():
    """Read arm claw coordinates from CSV file"""
    current_pos_csv = os.path.join("..", "csv", "coordinates_log.csv")
    current_x, current_y = None, None
    
    try:
        if os.path.exists(current_pos_csv):
            with open(current_pos_csv, mode="r") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    current_x = int(row['c1_c2_avg_x'].strip())
                    current_y = int(row['c1_c2_avg_y'].strip())
                    break  # Only read the first row
    except (FileNotFoundError, ValueError, KeyError):
        pass  # If file doesn't exist or has issues, arm coordinates remain None
    
    return current_x, current_y

def read_goal_position():
    """Read arm claw coordinates from CSV file"""
    goal_pos_csv = os.path.join("..", "csv", "coordinates_log.csv")
    goal_x, goal_y = None, None
    
    try:
        if os.path.exists(goal_pos_csv):
            with open(goal_pos_csv, mode="r") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    goal_x = int(row['c0_x'].strip())
                    goal_y = int(row['c0_y'].strip())
                    break  # Only read the first row
    except (FileNotFoundError, ValueError, KeyError):
        pass  # If file doesn't exist or has issues, arm coordinates remain None
    
    return goal_x, goal_y

def polar_from_points(a, b, c, d):
    dx = c - a
    dy = d - b
    rho = math.hypot(dx, dy)
    theta_deg = (math.degrees(math.atan2(-dy, dx)) + 360) % 360     # senza questo meno è antiorario, rifletto il sistema sull'asse x
    return rho, theta_deg

def angle_diff(goal, current):      # non ha senso ruotare servo di 180+ gradi si può ruotare nell'altro senso
    diff = (goal - current) % 360
    if diff > 180:
        diff -= 360
    return diff

def send_servo_command(servo_id, angle):
    try:
        r = requests.get(ESP32_URL, params={"servo": servo_id, "angle": angle}, timeout=0.3)
        print(f"Servo {servo_id} → {angle}° → {r.status_code}")
        return 0
    except Exception as e:
        print(f"Errore con servo {servo_id}: {e}")
        return -1
    
def joint1():
    arm_x, arm_y = read_arm_coordinates()
    last_servo_1_angle = OFFSET_SERVO1
    first_move_done = False
    last_goal_angle = None

    while True:
        current_x, current_y = read_current_position()
        goal_x, goal_y = read_goal_position()


        if None in (current_x, current_y, goal_x, goal_y):
            print("Coordinate non disponibili, skip...")
            time.sleep(0.5)
            continue
        
        current_rho, current_theta_deg = polar_from_points(arm_x, arm_y, current_x, current_y)
        print(f"Distanza current: {current_rho:.2f}, Angolo: {current_theta_deg:.2f}°")
        
        goal_rho, goal_theta_deg = polar_from_points(arm_x, arm_y, goal_x, goal_y)
        print(f"Distanza goal: {goal_rho:.2f}, Angolo: {goal_theta_deg:.2f}°")

        # reset one-shot se il goal cambia molto
        if last_goal_angle is None or abs(angle_diff(goal_theta_deg, last_goal_angle)) >= 5:
            first_move_done = False
        last_goal_angle = goal_theta_deg

        delta = angle_diff(goal_theta_deg, current_theta_deg)

        if abs(delta) > 2:
            if not first_move_done:
                move = angle_diff(goal_theta_deg, current_theta_deg)
                max_jump = 30  # limite massimo per il salto iniziale
                if abs(move) > max_jump:
                    move = max_jump if move > 0 else -max_jump
                target_angle = last_servo_1_angle + move
                while send_servo_command(1, round(target_angle)) < 0:
                    print("Ritento one-shot servo 1 →", round(target_angle))
                    time.sleep(2)
                last_servo_1_angle = target_angle
                first_move_done = True
            else:
                # piccoli step
                max_step = 2
                move = max_step if delta > max_step else (-max_step if delta < -max_step else delta)
                target_angle = last_servo_1_angle + move
                while send_servo_command(1, round(target_angle)) < 0:
                    print("Ritento step servo 1 →", round(target_angle))
                    time.sleep(2)
                last_servo_1_angle = target_angle
        else:
            print("Servo 1 in posizione")
            time.sleep(0.5)
            return

        time.sleep(0.8)

def joint2_3():
    # first set x,y to desired coordinates
    # then lower as much as needed
    return

def clap(condition):
    if condition == "open":
        send_servo_command(4, 175)
    elif condition == "close":
        send_servo_command(4, 140)      # 140 is full closed, it might use too much energy to force the servo 4, 150 or even more could be enought 
    return


if __name__ == "__main__":
    while True:
        clap("open")
        joint1()
        joint2_3()
        clap("close")