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
# Default settings
settings = {
    "attitude_frequency": 10,
    "reverse_roll": False,
    "reverse_pitch": False,
    "swap_pitch_roll": False,
    "reverse_yaw": False,
    "fixed_yaw_angle": None,
    "ws_url": "ws://18.234.27.121:8085",  # WebSocket URL for attitude data
    "marker_ws_url": "ws://18.234.27.121:8085",  # WebSocket URL for marker detection data
    "enable_attitude_control": True,
    "enable_marker_detection": True
}


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
print("TRYING TO CONNECT TO UDP PORT")
master.wait_heartbeat()

# Flush serial buffer if using serial connection
if hasattr(master, 'port') and hasattr(master.port, 'flushInput'):
    master.port.flushInput()

# WebSocket connection
ws = websocket.WebSocket()
ws.connect(settings["ws_url"])

marker_ws = websocket.WebSocket()
marker_ws.connect(settings["marker_ws_url"])

# Function to send landing target message via MAVLink
def send_landing_target(angle_x, angle_y, distance=0.0):
    master.mav.landing_target_send(
        int(time.time() * 1000000),
        0, mavutil.mavlink.MAV_FRAME_BODY_NED, angle_x, angle_y, distance_rangefinder, 0.0, 0.0, 0.0, 0.0, 0.0, [0.0, 0.0, 0.0, 0.0], 0, 0)

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
    elif len(angles) == 2:
        # If only angle_x and angle_y are provided, do nothing
        pass
    else:
        raise ValueError("Invalid number of arguments for send_ws_message()")

def request_message_interval(message_id: int, frequency_hz: float):
    print(f"Requesting message {message_id} at {frequency_hz} Hz")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, 1e6 / frequency_hz, 0, 0, 0, 0, 0
    )
    
distance_rangefinder = 0.0  # Default distance value

#def rangefinder_listener():
 #   global distance
#    while True:
 #       message = master.recv_match(type='DISTANCE_SENSOR', blocking=True)
 #       if message:
 #           distance = message.current_distance / 100.0  # Convert to meters if needed
## Attitude Control Logic
def attitude_control():
    global attitude_running, distance_rangefinder
    request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, settings['attitude_frequency'])
    request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 10)
    
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
            elif message and message.get_type() == 'DISTANCE_SENSOR':
                distance_rangefinder = message.current_distance / 100.0  # Distance in meters
                print(f"Rangefinder Distance: {distance_rangefinder} meters")
                
                
        except Exception as e:
            print(f"Error in attitude control: {e}")
            break

# Marker Detection Logic
def marker_detection():
    global marker_running, distance_rangefinder
    aruco_marker_image = cv2.imread('marker.jpg')
    if aruco_marker_image is None:
        print("Error: ArUco marker image not found.")
        return

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

                print(f"Marker Detected: {detected_marker_id}, Angular Offsets: angle_x={angle_x}, angle_y={angle_y}, Distance={distance_rangefinder}")

                # Send marker data over the second WebSocket
                marker_data = {
                    "markerId": int(detected_marker_id),  # Use standard int
                    "angle_x": round(float(angle_x), 3),  # Convert NumPy float to Python float
                    "angle_y": round(float(angle_y), 3),  # Convert NumPy float to Python float
                    "distance": round(float(distance_rangefinder), 3)  # Ensure distance is a standard float
                }
                send_landing_target(angle_x,angle_y)
                marker_ws.send(json.dumps(marker_data))
                print(f"Sent over marker WebSocket: {marker_data}")

    cap.release()


def handle_command(command):
    global attitude_running, marker_running
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
    elif command == "stop_marker":
        marker_running = False
        settings["enable_marker_detection"] = False  # Update setting
        save_config()  # Save updated setting to file
        print("Marker detection stopped.")
    elif command == "start_marker":
        if not marker_running:  # Only start if not already running
            marker_running = True
            settings["enable_marker_detection"] = True  # Update setting
            save_config()  # Save updated setting to file
            threading.Thread(target=marker_detection, daemon=True).start()  # Always start a new thread
            print("Marker detection started.")
    else:
        print(f"Unknown command: {command}")

# Main function
def main():
    load_config()

    server_socket = create_socket()
    if server_socket is None:
        return  # Exit if the socket creation failed
    
    global attitude_running, marker_running

    # Start the rangefinder listener thread
    #rangefinder_thread = threading.Thread(target=rangefinder_listener, daemon=True)
    #rangefinder_thread.start()
    
    # Start attitude and marker detection threads
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
        time.sleep(1)

if __name__ == "__main__":
    main()
