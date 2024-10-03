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
    "ws_url": "ws://18.234.27.121:8085",
    "enable_attitude_control": True,
    "enable_marker_detection": True
}

# Global control flags for the threads
attitude_running = False
marker_running = False

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

# MAVLink connection
master = mavutil.mavlink_connection("/dev/serial0", baud=921600)
master.wait_heartbeat()

# Flush serial buffer if using serial connection
if hasattr(master, 'port') and hasattr(master.port, 'flushInput'):
    master.port.flushInput()

# WebSocket connection
ws = websocket.WebSocket()
ws.connect(settings["ws_url"])

# Function to send landing target message via MAVLink
def send_landing_target(angle_x, angle_y, distance=0.0):
    master.mav.landing_target_send(
        int(time.time() * 1000000),
        0, mavutil.mavlink.MAV_FRAME_BODY_NED, angle_x, angle_y, distance, 0.0, 0.0, 0.0, 0.0, 0.0, [0.0, 0.0, 0.0, 0.0], 0, 0)

# Function to send angular offsets over WebSocket
def send_ws_message(*angles):
    if len(angles) == 3:
        yaw, pitch, roll = angles
        timestamp = int(time.time() * 1000)
        data = {
            "values": [round(yaw, 3), round(pitch, 3), round(roll, 3)],
            "timestamp": timestamp,
            "accuracy": 3
        }
        message = json.dumps(data)
        ws.send(message)
        print(f"Sent over WebSocket: {message}")

def request_message_interval(message_id: int, frequency_hz: float):
    print(f"Requesting message {message_id} at {frequency_hz} Hz")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, 1e6 / frequency_hz, 0, 0, 0, 0, 0
    )

# Attitude Control Logic
def attitude_control():
    global attitude_running
    request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, settings['attitude_frequency'])

    while attitude_running:
        try:
            load_config()
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

                send_ws_message(yaw, pitch, roll)
        except Exception as e:
            print(f"Error in attitude control: {e}")
            break

# Marker Detection Logic
def marker_detection():
    global marker_running
    aruco_marker_image = cv2.imread('marker.jpg')
    if aruco_marker_image is None:
        print("Error: ArUco marker image not found.")
        return

    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
    parameters = aruco.DetectorParameters_create()

    cap = cv2.VideoCapture(VIDEO_URL)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, PROCESSING_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, PROCESSING_HEIGHT)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    frn = 0
    while marker_running:
        cap.grab()
        ret, frame = cap.retrieve()

        if not ret or frame is None:
            print("Error: Frame not retrieved.")
            break

        frn += 1
        if frn % PROCESSING_INTERVAL != 0:
            continue

        corners, ids, rejected = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

        if ids is not None:
            for i in range(len(ids)):
                detected_marker_id = ids[i][0]
                corner = corners[i].reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corner

                x1, y1 = topLeft
                x2, y2 = bottomRight
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)

                image_center_x = frame.shape[1] // 2
                image_center_y = frame.shape[0] // 2

                angle_x = ((center_x - image_center_x) / PROCESSING_WIDTH) * FOV_X
                angle_y = ((center_y - image_center_y) / PROCESSING_HEIGHT) * FOV_Y

                print(f"Marker Detected: {detected_marker_id}, Angular Offsets: angle_x={angle_x}, angle_y={angle_y}")
                send_landing_target(angle_x, angle_y)

    cap.release()

# Main function
def main():
    load_config()

    global attitude_running, marker_running

    if settings["enable_attitude_control"]:
        attitude_running = True
        attitude_thread = threading.Thread(target=attitude_control)
        attitude_thread.daemon = True
        attitude_thread.start()

    if settings["enable_marker_detection"]:
        marker_running = True
        marker_thread = threading.Thread(target=marker_detection)
        marker_thread.daemon = True
        marker_thread.start()

    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()
