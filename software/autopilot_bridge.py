# autopilot_bridge.py
# Simple bridge: send ARM, SET commands to Arduino autopilot via serial.
# Also shows how to compute a route and step through waypoints (no DroneKit required for bench test)

import serial, time
from route_planning_advanced import a_star_grid
import sys

SERIAL_PORT = "COM4"    # change to your Arduino port e.g. /dev/ttyUSB0 on Linux or COM3 on Windows
BAUD = 115200

def open_serial(port=SERIAL_PORT, baud=BAUD, timeout=1):
    s = serial.Serial(port, baud, timeout=0.2)
    time.sleep(2)
    return s

def arm_drone(serial_conn):
    serial_conn.write(b"ARM\\n")
    time.sleep(0.2)
    resp = serial_conn.read_all().decode('utf-8', errors='ignore')
    print("ARDUINO:", resp.strip())

def disarm_drone(serial_conn):
    serial_conn.write(b"DISARM\\n")
    time.sleep(0.2)
    print("Sent DISARM")

def setpoints(serial_conn, roll_deg, pitch_deg, yaw_deg, throttle_us):
    cmd = f"SET {roll_deg:.2f} {pitch_deg:.2f} {yaw_deg:.2f} {int(throttle_us)}\\n"
    serial_conn.write(cmd.encode('utf-8'))
    time.sleep(0.01)

def demo_route_and_bridge(center, start, goal):
    print("Calculating route...")
    path = a_star_grid(start, goal, center, radius_m=500, step_m=20)
    if not path:
        print("No path found")
        return
    print("Waypoints count:", len(path))
    for wp in path:
        print("Waypoint:", wp)
    # run through waypoints and send simplistic commands to Arduino (no real guidance)
    try:
        ser = open_serial()
    except Exception as e:
        print("Failed to open serial:", e)
        return
    arm_drone(ser)
    # simple demo: for each waypoint, set small pitch/roll offsets and moderate throttle
    for wp in path:
        # compute rough offset to goal to set pitch/roll (toy demo)
        # In a real autopilot you'd use GPS-to-heading and closed-loop pos controller
        setpoints(ser, 0.0, 0.0, 0.0, 1400)  # keep flat; throttle 1400
        time.sleep(1.0)
    # finish
    print("Demo done. Disarming.")
    disarm_drone(ser)
    ser.close()

if __name__ == "__main__":
    # test run with dummy coords near Dhanbad center - replace with real
    center = (23.8, 86.4)
    start = (23.800, 86.400)
    goal = (23.804, 86.404)
    demo_route_and_bridge(center, start, goal)
