import requests
import time

# Configura l'indirizzo IP dell'ESP32
ESP32_URL = "http://192.168.4.1/set"

# Sequenza di angoli da testare (puoi personalizzarla)
angles_sequence = [
    [90, 126, 150, 160],     # posizione neutra
    [40, 110, 150, 142],   # mossa 1
    [60, 140, 130, 160],   # mossa 2
    [90, 130, 150, 145]      # ritorno a neutro
]


def send_servo_command(servo_id, angle):
    try:
        r = requests.get(ESP32_URL, params={"servo": servo_id, "angle": angle}, timeout=0.3)
        print(f"Servo {servo_id} → {angle}° → {r.status_code}")
    except Exception as e:
        print(f"Errore con servo {servo_id}: {e}")

def run_sequence():
    while True:
        for step in angles_sequence:
            for i, angle in enumerate(step):
                send_servo_command(i + 1, angle)  # servo_id = 1 to 4
                time.sleep(1)

def run_demo():
    while True:
        send_servo_command(4, 160)
        time.sleep(2)

        send_servo_command(1, 40)
        time.sleep(2)
        send_servo_command(3, 150)
        time.sleep(7)
        send_servo_command(2, 130)
        time.sleep(2)
        send_servo_command(4, 140)
        time.sleep(3)

        send_servo_command(3, 180)
        time.sleep(2)
        send_servo_command(1, 300)
        time.sleep(12)
        send_servo_command(2, 200)
        time.sleep(3)
        send_servo_command(4, 160)
        time.sleep(7)
        send_servo_command(2, 110)
        time.sleep(6)



if __name__ == "__main__":
    run_demo()