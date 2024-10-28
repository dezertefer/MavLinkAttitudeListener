import time
import math
import threading
import cv2
import cv2.aruco as aruco
import numpy as np
from pymavlink import mavutil
import json
import os
import socket

# Path for UNIX domain socket file
SOCKET_PATH = "/tmp/attitudeForward.sock"
config_file_path = "/home/cdc/MavLinkAttitudeListener/config.json"

# Constants for ArUco detection
PROCESSING_WIDTH = 320
PROCESSING_HEIGHT = 320
VIDEO_URL = 'rtsp://127.0.0.1:8554/cam'
FOV_X = 4.18879  # 240 degrees in radians
FOV_Y = 4.18879  # 240 degrees in radians
PROCESSING_INTERVAL = 1

# Default settings
settings = {
    "attitude_frequency": 10,
    "reverse_roll": False,
    "reverse_pitch": False,
    "swap_pitch_roll": False,
    "reverse_yaw": False,
    "fixed_yaw_angle": None,
}

# Create a UDP socket
UDP_IP = "127.0.0.1"
UDP_PORT = 13370
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def create_socket():
    if os.path.exists(SOCKET_PATH):
        print("Removing existing socket")
        os.remove(SOCKET_PATH)  # Remove the socket if it already exists
    try:
        print("Creating socket...")
        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        server.bind(SOCKET_PATH)
        server.listen(1)  # Listen for incoming connections
        print(f"Socket created and listening at {SOCKET_PATH}")
        return server
    except socket.error as e:
        print(f"Socket creation failed: {e}")
        return None

# Global control flags for the threads
attitude_running = False

def load_config():
    global settings
    if os.path.exists(config_file_path):
        with open(config_file_path, 'r') as file:
            settings.update(json.load(file))

# Save settings to JSON
def save_config():
    with open(config_file_path, 'w') as file:
        json.dump(settings, file, indent=4)
    print("Configuration saved:", settings)

load_config()
# MAVLink connection
master = mavutil.mavlink_connection('udpin:localhost:14550')
print("TRYING TO CONNECT TO TCP PORT")
master.wait_heartbeat()

# Function to send attitude messages over UDP
def send_udp_message(yaw, pitch, roll):
    message = {
        "values": [round(yaw, 3), round(pitch, 3), round(roll, 3)],
        "timestamp": int(time.time() * 1000),
        "accuracy": 3
    }
    message_json = json.dumps(message)
    sock.sendto(message_json.encode(), (UDP_IP, UDP_PORT))
    print(f"Sent message to {UDP_IP}:{UDP_PORT}: {message_json}")

# Attitude Control Logic
def attitude_control():
    global attitude_running
    print("Starting attitude control...")
    while attitude_running:
        try:
            message = master.recv_match(blocking=True)
            if message and message.get_type() == 'ATTITUDE':
                roll = math.degrees(message.roll)
                pitch = math.degrees(message.pitch)
                yaw = math.degrees(message.yaw)

                if settings["reverse_roll"]:
                    roll = -roll
                if settings["reverse_pitch"]:
                    pitch = -pitch
                if settings["reverse_yaw"]:
                    yaw = -yaw
                if settings["fixed_yaw_angle"] is not None:
                    yaw = settings["fixed_yaw_angle"]
                if settings["swap_pitch_roll"]:
                    roll, pitch = pitch, roll
                
                send_udp_message(yaw, pitch, roll)
            time.sleep(0.1)

        except Exception as e:
            print(f"Error in attitude control: {e}")
            break

def handle_command(command):
    global attitude_running
    if command == "stop_attitude":
        attitude_running = False
        settings["enable_attitude_control"] = False  # Update setting
        save_config()  # Save updated setting to file
        print("Attitude control stopped.")
    elif command == "start_attitude":
        if not attitude_running:  # Only start if not already running
            attitude_running = True
            settings["enable_attitude_control"] = True  # Update setting
            save_config()  # Save updated setting to file
            threading.Thread(target=attitude_control, daemon=True).start()  # Always start a new thread
            print("Attitude control started.")
    else:
        print(f"Unknown command: {command}")

# Main function
def main():
    load_config()

    server_socket = create_socket()
    if server_socket is None:
        return  # Exit if the socket creation failed

    # Start attitude control if enabled
    if settings.get("enable_attitude_control", False):
        attitude_running = True
        threading.Thread(target=attitude_control, daemon=True).start()

    while True:
        conn, _ = server_socket.accept()
        command = conn.recv(1024).decode().strip()
        handle_command(command)
        conn.close()
        time.sleep(0.001)

if __name__ == "__main__":
    main()
