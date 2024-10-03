import time
import math
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
    "ws_url": "ws://18.234.27.121:8085",
    "enable_attitude_control": True,
    "enable_marker_detection": True
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

# WebSocket connection
ws = websocket.WebSocket()
ws.connect(settings["ws_url"])

# MAVLink and WebSocket message functions
def send_landing_target(angle_x, angle_y, distance=0.0):
    """Send LANDING_TARGET message to ArduPilot"""
    master.mav.landing_target_send(
        int(time.time() * 1000000),
        0, mavutil.mavlink.MAV_FRAME_BODY_NED, angle_x, angle_y, distance, 0.0, 0.0, 0.0, 0.0, 0.0, [0.0, 0.0, 0.0, 0.0], 0, 0)

def send_ws_message(*angles):
    """Send angular offsets over WebSocket"""
    if len(angles) == 3:
        yaw, pitch, roll = angles
        message = f"Yaw: {yaw:.2f}, Pitch: {pitch:.2f}, Roll: {roll:.2f}"
    elif len(angles) == 2:
        angle_x, angle_y = angles
        message = f"Xangle: {angle_x:.2f}, Yangle: {angle_y:.2f}"
    else:
        raise ValueError("Invalid number of arguments for send_ws_message()")
    
    ws.send(message)
    print(f"Sent over WebSocket: {message}")

def request_message_interval(message_id: int, frequency_hz: float):
    """Request MAVLink message at a desired frequency."""
    print(f"Requesting message {message_id} at {frequency_hz} Hz")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, 1e6 / frequency_hz, 0, 0, 0, 0, 0
    )

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
    print("Marker detection started.")  # Debugging statement
    cap = cv2.VideoCapture(VIDEO_URL)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, PROCESSING_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, PROCESSING_HEIGHT)

    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
    parameters = aruco.DetectorParameters_create()

    while settings["enable_marker_detection"]:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("Error: Could not read video frame.")
            continue

        print("Frame captured.")  # Debugging statement
        corners, ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters)
        if ids is not None:
            print(f"Markers detected: {ids}")  # Debugging statement
            for i in range(len(ids)):
                corner = corners[i][0]
                center_x = (corner[0][0] + corner[2][0]) / 2
                center_y = (corner[0][1] + corner[2][1]) / 2
                angle_x = ((center_x - PROCESSING_WIDTH / 2) / PROCESSING_WIDTH) * FOV_X
                angle_y = ((center_y - PROCESSING_HEIGHT / 2) / PROCESSING_HEIGHT) * FOV_Y
                send_landing_target(angle_x, angle_y)
                send_ws_message(angle_x, angle_y)
        else:
            print("No markers detected.")  # Debugging statement

# Main loop without threading
def main():
    load_config()
    if settings["enable_attitude_control"]:
        attitude_control()

    if settings["enable_marker_detection"]:
        marker_detection()

if __name__ == "__main__":
    main()
