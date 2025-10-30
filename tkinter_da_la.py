import time
import csv
import serial
from datetime import datetime
from ydlidar_x2 import YDLidarX2

# --- Serial Motor Communication ---
motor = serial.Serial(port='COM16', baudrate=115200, timeout=1)  # Change COM port

def send_motor_cmd(cmd, expected='ok'):
    """Send command to motor and check for expected response"""
    motor.write(cmd.encode())
    time.sleep(0.05)
    resp = motor.readline().decode().strip()
    if expected not in resp:
        print(f"[Motor] Sent: {cmd} | Got: {resp}")
    return resp

# --- LiDAR Setup ---
lidar = YDLidarX2(port='COM8')  # Change to your correct port
if not lidar.connect():
    print("Failed to connect to LiDAR.")
    exit(1)

lidar.start_scan()
print("✅ LiDAR started...")

time.sleep(2);
# --- Motor Init ---
send_motor_cmd('R', expected='rc_ok')  # Arm motor

# --- CSV File Setup (scan.py compatible) ---
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
csv_path = f'./CSV/scanData_{timestamp}.csv'
csv_file = open(csv_path, 'w', newline='')
writer = csv.writer(csv_file)
writer.writerow(['Quality', 'Angle (degrees)', 'Distance (mm)', 'Rotation'])

# --- Scan Configuration ---
steps = 400  # 180° / 0.45° = 400 steps
step_angle = 180.0 / steps
scans_per_step = 2  # Oversampling



try:
    for step in range(steps):
        rotation_deg = step * step_angle
        print(f"\n[STEP {step}/{steps}] Rotation = {rotation_deg:.2f}°")

        seen = set()
        scans_collected = 0

        while scans_collected < scans_per_step:
            if not lidar.available:
                time.sleep(0.01)
                continue

            data = lidar.get_data()

            for angle in range(180):  # Only 0° to 179°
                distance = data[angle]
                if distance == lidar.out_of_range:
                    continue

                measurement_id = (angle, distance)
                if measurement_id not in seen:
                    seen.add(measurement_id)
                    writer.writerow([1, angle, distance, rotation_deg])
            scans_collected += 1

        send_motor_cmd('C')  # Step motor CW
        time.sleep(0.05)

    print("✅ Scan complete. Data saved.")

finally:
    # Cleanup
    lidar.stop_scan()
    lidar.disconnect()
    send_motor_cmd('D', expected='rc_ok')  # Disarm
    csv_file.close()
    print("🛑 LiDAR stopped, motor disarmed.")
    print(f"📁 Data saved to: {csv_path}")
