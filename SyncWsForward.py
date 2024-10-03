import time
import math
import threading
import websocket
import cv2
import cv2.aruco as aruco
import numpy as np
from pymavlink import mavutil
import json
import os
import sys
import socket

# Path for UNIX domain socket file
SOCKET_PATH = "/tmp/attitudeForward.sock"
config_file_path = "/home/cdc/MavLinkAttitudeListener/config.json" 

# Constants for ArUco detection
PROCESSING_WIDTH = 320
PROCESSING_HEIGHT = 320
VIDEO_URL = 'rtsp://127.0.0.1:8554/cam'
PROCESSING_INTERVAL = 1
FOV_X = 4.18879  # 240 degrees in radians
FOV_Y = 4.18879  # 240 degrees in radians

# Default settings
settings = {
    "attitude_frequency": 10,
    "reverse_roll": False,
    "reverse_pitch": False,
    "swap_pitch_roll": False,
    "reverse_yaw": False,
    "fixed_yaw_angle": None,
    "ws_url": "ws://18.234.27.121:8085",  # Default WebSocket URL
    "enable_attitude_control": True,  # To control enabling/disabling of attitude
    "enable_marker_detection": True  # To control enabling/disabling of marker detection
}

# Load settings from JSON
def load_config():
    global settings
    if os.path.exists(config_file_path):
        with open(config_file_path, 'r') as file:
            settings.update(json.load(file))
            print("Configuration reloaded:", settings)
    else:
        print("Configuration file not found, using default settings.")

# Save settings to JSON
def save_config():
    with open(config_file_path, 'w') as file:
        json.dump(settings, file, indent=4)
    print("Configuration saved:", settings)

# Load settings at the start
load_config()

# Create the MAVLink connection
master = mavutil.mavlink_connection("/dev/serial0", baud=921600)
master.wait_heartbeat()

# Flush serial buffer if using serial connection
if hasattr(master, 'port') and hasattr(master.port, 'flushInput'):
    master.port.flushInput()  # Flush the serial input buffer

# Create a UNIX socket
def create_socket():
    if os.path.exists(SOCKET_PATH):
        os.remove(SOCKET_PATH)
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(SOCKET_PATH)
    server.listen(1)
    print(f"Socket created and listening at {SOCKET_PATH}")
    return server

# WebSocket connection
ws = websocket.WebSocket()
ws.connect(settings["ws_url"])

# MAVLink and WebSocket message functions
def send_landing_target(angle_x, angle_y, distance=0.0):
    """Send LANDING_TARGET message to ArduPilot"""
    master.mav.landing_target_send(
        int(time.time() * 1000000),
        0, mavutil.mavlink.MAV_FRAME_BODY_NED, angle_x, angle_y, distance, 0.0, 0.0, 0.0, 0.0, 0.0, [0.0, 0.0, 0.0, 0.0], 0, 0)

def send_ws_message(angle_x, angle_y):
    """Send angular offsets over WebSocket"""
    message = f"Xangle: {angle_x:.2f}, Yangle: {angle_y:.2f}"
    ws.send(message)
    print(f"Sent over WebSocket: {message}")

# Attitude Control Logic
def attitude_control():
    request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, settings['attitude_frequency'])

    while settings["enable_attitude_control"]:
        try:
            load_config()  # Reload settings dynamically
            message = master.recv_match(blocking=True)

            if message and message.get_type() == 'ATTITUDE':
                roll = math.degrees(message.roll)
                pitch = math.degrees(message.pitch)
                yaw = math.degrees(message.yaw)

                # Apply settings
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

                send_ws_message(yaw, pitch, roll)
        except Exception as e:
            print(f"Error in attitude control: {e}")
            break

# Marker Detection Logic
def marker_detection():
    cap = cv2.VideoCapture(VIDEO_URL)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, PROCESSING_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, PROCESSING_HEIGHT)

    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
    parameters = aruco.DetectorParameters_create()

    while settings["enable_marker_detection"]:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read video frame.")
            continue

        corners, ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters)
        if ids is not None:
            for i in range(len(ids)):
                corner = corners[i][0]
                center_x = (corner[0][0] + corner[2][0]) / 2
                center_y = (corner[0][1] + corner[2][1]) / 2
                angle_x = ((center_x - PROCESSING_WIDTH / 2) / PROCESSING_WIDTH) * FOV_X
                angle_y = ((center_y - PROCESSING_HEIGHT / 2) / PROCESSING_HEIGHT) * FOV_Y
                send_landing_target(angle_x, angle_y)
                send_ws_message(angle_x, angle_y)

# Socket handler to start/stop attitude or marker detection
def handle_command(command):
    global settings
    if command == "stop_attitude":
        settings["enable_attitude_control"] = False
    elif command == "start_attitude":
        settings["enable_attitude_control"] = True
        threading.Thread(target=attitude_control).start()
    elif command == "stop_marker":
        settings["enable_marker_detection"] = False
    elif command == "start_marker":
        settings["enable_marker_detection"] = True
        threading.Thread(target=marker_detection).start()
    save_config()

# Main thread to listen to socket commands
def main():
    server_socket = create_socket()
    threading.Thread(target=attitude_control).start()  # Start attitude control
    threading.Thread(target=marker_detection).start()  # Start marker detection

    while True:
        conn, _ = server_socket.accept()
        with conn:
            command = conn.recv(1024).decode().strip()
            if command:
                handle_command(command)

if __name__ == "__main__":
    main()
