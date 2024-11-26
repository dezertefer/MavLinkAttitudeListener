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

UDP_IP = "127.0.0.1"  # Client address
UDP_PORT = 13370    # Client port to send data to

# Constants for ArUco detection
PROCESSING_WIDTH = 320
PROCESSING_HEIGHT = 320
VIDEO_URL = 'rtsp://127.0.0.1:8554/cam'
FOV_X = 4.18879  # 240 degrees in radians
FOV_Y = 4.18879  # 240 degrees in radians
PROCESSING_INTERVAL = 1

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Global control flags for the threads
attitude_running = False
marker_running = False

# Load settings only once
settings = {
    "attitude_frequency": 10,
    "reverse_roll": False,
    "reverse_pitch": False,
    "swap_pitch_roll": False,
    "reverse_yaw": False,
    "fixed_yaw_angle": None,
    "enable_attitude_control": True,
    "enable_marker_detection": True
}

if os.path.exists(config_file_path):
    with open(config_file_path, 'r') as file:
        settings.update(json.load(file))
    print("Configuration loaded:", settings)

# MAVLink connection
master = mavutil.mavlink_connection('udpin:localhost:14550')
print("TRYING TO CONNECT TO TCP PORT")
master.wait_heartbeat()

# Flush serial buffer if using serial connection
if hasattr(master, 'port') and hasattr(master.port, 'flushInput'):
    master.port.flushInput()

# Function to send UDP messages
def send_udp_message(data):
    message_json = json.dumps(data)
    sock.sendto(message_json.encode(), (UDP_IP, UDP_PORT))
    print(f"Sent message to {UDP_IP}:{UDP_PORT}: {message_json}")

# MAVLink request for message intervals
def request_message_interval(message_id: int, frequency_hz: float):
    print(f"Requesting message {message_id} at {frequency_hz} Hz")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, 1e6 / frequency_hz, 0, 0, 0, 0, 0
    )

distance_rangefinder = 0.0  # Default distance value

# Attitude Control Logic
def attitude_control():
    global attitude_running
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

                send_udp_message({"type": "attitude", "values": [round(yaw, 3), round(pitch, 3), round(roll, 3)], "timestamp": int(time.time() * 1000)})
        except Exception as e:
            print(f"Error in attitude control: {e}")
            break

# Marker Detection Logic
def marker_detection():
    global marker_running, distance_rangefinder
    request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, settings['attitude_frequency'])

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
            continue

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

                marker_data = {
                    "type": "marker",
                    "markerId": int(detected_marker_id),
                    "angle_x": round(float(angle_x), 3),
                    "angle_y": round(float(angle_y), 3),
                    "distance": round(float(distance_rangefinder), 3)
                }
                send_landing_target(angle_x,angle_y)
                send_udp_message(marker_data)

    cap.release()

def send_landing_target(angle_x, angle_y, distance=0.0):
    master.mav.landing_target_send(
        int(time.time() * 1000000),
        0, mavutil.mavlink.MAV_FRAME_BODY_NED, angle_x, angle_y, distance_rangefinder, 0.0, 0.0, 0.0, 0.0, 0.0, [0.0, 0.0, 0.0, 0.0], 0, 0)

def handle_command(command):
    global attitude_running, marker_running
    if command == "stop_attitude":
        attitude_running = False
        print("Attitude control stopped.")
    elif command == "start_attitude":
        if not attitude_running:
            attitude_running = True
            threading.Thread(target=attitude_control, daemon=True).start()
            print("Attitude control started.")
    elif command == "stop_marker":
        marker_running = False
        print("Marker detection stopped.")
    elif command == "start_marker":
        if not marker_running:
            marker_running = True
            threading.Thread(target=marker_detection, daemon=True).start()
            print("Marker detection started.")
    else:
        print(f"Unknown command: {command}")

def create_socket():
    if os.path.exists(SOCKET_PATH):
        os.remove(SOCKET_PATH)
    try:
        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        server.bind(SOCKET_PATH)
        server.listen(1)
        return server
    except socket.error as e:
        print(f"Socket creation failed: {e}")
        return None

# Main function
def main():
    server_socket = create_socket()
    if server_socket is None:
        return

    global attitude_running, marker_running

    if settings["enable_attitude_control"]:
        attitude_running = True
        threading.Thread(target=attitude_control, daemon=True).start()

    if settings["enable_marker_detection"]:
        marker_running = True
        threading.Thread(target=marker_detection, daemon=True).start()

    while True:
        conn, _ = server_socket.accept()
        command = conn.recv(1024).decode().strip()
        handle_command(command)
        conn.close()
        time.sleep(0.001)

if __name__ == "__main__":
    main()
