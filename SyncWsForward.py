import time
import math
import threading  # Import threading module
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
def create_socket():
    if os.path.exists(SOCKET_PATH):
        os.remove(SOCKET_PATH)  # Remove the socket if it already exists
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(SOCKET_PATH)
    server.listen(1)  # Listen for incoming connections
    return server

# Load settings from JSON
def load_config():
    global settings
    if os.path.exists(config_file_path):
        with open(config_file_path, 'r') as file:
            settings.update(json.load(file))
#            print("Configuration reloaded:", settings)

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

def marker_detection():
    # Preload ArUco marker image for testing
    aruco_marker_image = cv2.imread('marker.jpg')
    if aruco_marker_image is None:
        print("Error: ArUco marker image not found.")
        return

    # Load ArUco dictionary and parameters
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
    parameters = aruco.DetectorParameters_create()

    print('Loading the pre-defined ArUco marker...')

    # Create a margin around the pre-defined marker
    margin_size = 20
    color = [255, 255, 255]  # White color for margin

    # Get original dimensions
    height, width, channels = aruco_marker_image.shape

    # Create a new image with margin
    new_height = height + 2 * margin_size
    new_width = width + 2 * margin_size
    image_with_margin = np.full((new_height, new_width, channels), color, dtype=np.uint8)

    # Place the original marker in the center of the new image
    image_with_margin[margin_size:margin_size + height, margin_size:margin_size + width] = aruco_marker_image
    corners, ids, rejected = aruco.detectMarkers(image_with_margin, aruco_dict, parameters=parameters)

    # Check if a valid marker was found in the pre-loaded image
    if ids is None:
        print('Error: Invalid ArUco marker in the pre-loaded image.')
        return
    else:
        print(f"Pre-loaded marker detected, ID: {ids[0][0]}")

    # Continue with live video stream detection
    cap = cv2.VideoCapture(VIDEO_URL)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, PROCESSING_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, PROCESSING_HEIGHT)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    frn = 0
    while True:
        cap.grab()
        ret, frame = cap.retrieve()

        if not ret or frame is None:
            print("Error: Frame not retrieved.")
            break

        frn += 1
        if frn % PROCESSING_INTERVAL != 0:
            continue

        # Detect ArUco markers in the video frame
        corners, ids, rejected = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

        if ids is not None:
            for i in range(len(ids)):
                detected_marker_id = ids[i][0]

                # Get the corner coordinates
                corner = corners[i].reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corner

                x1, y1 = topLeft
                x2, y2 = bottomRight
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)

                image_center_x = frame.shape[1] // 2
                image_center_y = frame.shape[0] // 2

                # Calculate angular offsets
                angle_x = ((center_x - image_center_x) / PROCESSING_WIDTH) * FOV_X
                angle_y = ((center_y - image_center_y) / PROCESSING_HEIGHT) * FOV_Y

                print(f"Marker Detected in video: {detected_marker_id}, Angular Offsets: angle_x={angle_x}, angle_y={angle_y}")

                # Send MAVLink and WebSocket messages
                send_landing_target(angle_x, angle_y)
                angle_X = math.degrees(angle_x)
                angle_Y = math.degrees(angle_y)
                send_ws_message(angle_X, angle_Y)

    # Release resources
    cap.release()

# Main loop using threading to run both attitude control and marker detection concurrently
def main():
    load_config()
    
    # Ensure the socket is created
    server_socket = create_socket()
    if server_socket is None:
        return  # Exit if the socket creation failed

    # Create a thread for attitude control
    if settings["enable_attitude_control"]:
        attitude_thread = threading.Thread(target=attitude_control)
        attitude_thread.daemon = True  # Allow the program to exit even if this thread is running
        attitude_thread.start()

    # Create a thread for marker detection
    if settings["enable_marker_detection"]:
        marker_thread = threading.Thread(target=marker_detection)
        marker_thread.daemon = True  # Allow the program to exit even if this thread is running
        marker_thread.start()

    # Keep the main program running while the threads do the work
    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()
