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

# Default settings
settings = {
    "attitude_frequency": 10,
    "reverse_roll": False,
    "reverse_pitch": False,
    "swap_pitch_roll": False,
    "reverse_yaw": False,
    "fixed_yaw_angle": None,
    "enable_marker_detection": True
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

load_config()
# MAVLink connection
master = mavutil.mavlink_connection('udpin:localhost:14550')
print("TRYING TO CONNECT TO TCP PORT")
master.wait_heartbeat()
print("Connected to MAVLink")

def send_udp_message(data):
    message_json = json.dumps(data)
    sock.sendto(message_json.encode(), (UDP_IP, UDP_PORT))
    print(f"Sent message to {UDP_IP}:{UDP_PORT}: {message_json}")

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
    print("Starting attitude control...")
    request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, settings['attitude_frequency'])

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
                
                send_udp_message({"type": "attitude", "values": [round(yaw, 3), round(pitch, 3), round(roll, 3)], "timestamp": int(time.time() * 1000), "accuracy": 3})

            time.sleep(0.1)  # Adjust the sleep time as needed

        except Exception as e:
            print(f"Error in attitude control: {e}")
            break
    print("Attitude control stopped.")

# Marker Detection Logic
def marker_detection():
    global marker_running
    print('Loading the pre-defined ArUco marker...')
    aruco_marker_image = cv2.imread('marker.jpg')
    if aruco_marker_image is None:
        print("Error: ArUco marker image not found.")
        return

    print("Pre-defined ArUco marker loaded successfully.")

    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
    parameters = aruco.DetectorParameters_create()

    cap = cv2.VideoCapture(VIDEO_URL)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, PROCESSING_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, PROCESSING_HEIGHT)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    while marker_running:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("Error: Frame not retrieved.")
            break

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

                # Send marker data over UDP
                marker_data = {
                    "type": "marker",
                    "markerId": detected_marker_id,
                    "angle_x": round(float(angle_x), 3),
                    "angle_y": round(float(angle_y), 3),
                    "timestamp": int(time.time() * 1000)
                }
                send_udp_message(marker_data)

        time.sleep(0.1)

    cap.release()
    print("Marker detection stopped.")

def handle_command(command):
    global attitude_running, marker_running
    if command == "stop_attitude":
        attitude_running = False
        settings["enable_attitude_control"] = False
        save_config()
        print("Attitude control stopped.")
    elif command == "start_attitude":
        if not attitude_running:
            attitude_running = True
            settings["enable_attitude_control"] = True
            save_config()
            print("Starting attitude control thread...")
            threading.Thread(target=attitude_control, daemon=True).start()
            print("Attitude control started.")
    elif command == "stop_marker":
        marker_running = False
        settings["enable_marker_detection"] = False
        save_config()
        print("Marker detection stopped.")
    elif command == "start_marker":
        if not marker_running:
            marker_running = True
            settings["enable_marker_detection"] = True
            save_config()
            print("Starting marker detection thread...")
            threading.Thread(target=marker_detection, daemon=True).start()
            print("Marker detection started.")
    else:
        print(f"Unknown command: {command}")

# Main function
def main():
    load_config()

    server_socket = create_socket()
    if server_socket is None:
        return  # Exit if the socket creation failed

    # Start attitude and marker detection if enabled
    if settings.get("enable_attitude_control", False):
        attitude_running = True
        threading.Thread(target=attitude_control, daemon=True).start()
        print("Attitude control thread initiated.")

    if settings.get("enable_marker_detection", False):
        marker_running = True
        threading.Thread(target=marker_detection, daemon=True).start()
        print("Marker detection thread initiated.")

    while True:
        conn, _ = server_socket.accept()
        command = conn.recv(1024).decode().strip()
        handle_command(command)
        conn.close()
        time.sleep(0.001)

if __name__ == "__main__":
    main()
