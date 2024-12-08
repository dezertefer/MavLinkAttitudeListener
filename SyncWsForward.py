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
import logging

# Configure the logging system
# logging.basicConfig(
#    filename='log.txt',
#    level=logging.INFO,  # Set the logging level to INFO
#    format='%(asctime)s - %(levelname)s - %(message)s'  # Define the log message format
# )

# Path for UNIX domain socket file
SOCKET_PATH = "/tmp/attitudeForward.sock"
config_file_path = "/home/cdc/MavLinkAttitudeListener/config.json"

UDP_IP = "127.0.0.1"  # Client address
UDP_PORT = 14440    # Client port to send data to

# Constants for ArUco detection
PROCESSING_WIDTH = 200
PROCESSING_HEIGHT = 200
VIDEO_URL = 'rtsp://127.0.0.1:8554/cam'
FOV_X = 4.18879  # 240 degrees in radians
FOV_Y = 4.18879  # 240 degrees in radians
PROCESSING_INTERVAL = 1

MARKER_SWITCH_FRAME_COUNT = 75
MEDIAN_FILTER_WIN_SIZE = 9

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Default settings
# Default settings
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

# Flush serial buffer if using serial connection
if hasattr(master, 'port') and hasattr(master.port, 'flushInput'):
    master.port.flushInput()

# # WebSocket connection
# ws = websocket.WebSocket()
# ws.connect(settings["ws_url"])

# marker_ws = websocket.WebSocket()
# marker_ws.connect(settings["marker_ws_url"])

# Function to send landing target message via MAVLink
def send_udp_message(data):
    message_json = json.dumps(data)
    sock.sendto(message_json.encode(), (UDP_IP, UDP_PORT))
#    print(f"Sent message to {UDP_IP}:{UDP_PORT}: {message_json}")

def send_landing_target(angle_x, angle_y, distance=0.0):
    master.mav.landing_target_send(
        int(time.time() * 1000000),
        0, mavutil.mavlink.MAV_FRAME_BODY_NED, angle_x, angle_y, distance_rangefinder, 0.0, 0.0, 0.0, 0.0, 0.0, [0.0, 0.0, 0.0, 0.0], 0, 0)

# Function to send angular offsets over WebSocket
# def send_ws_message(*angles):
#     if len(angles) == 3:
#         yaw, pitch, roll = angles
#         timestamp = int(time.time() * 1000)
#         data = {
#             "values": [round(yaw, 3), round(pitch, 3), round(roll, 3)],
#             "timestamp": timestamp,
#             "accuracy": 3
#         }
#         message = json.dumps(data)
#         ws.send(message)
#  #       print(f"Sent over WebSocket: {message}")
#     elif len(angles) == 2:
#         # If only angle_x and angle_y are provided, do nothing
#         pass
#     else:
#         raise ValueError("Invalid number of arguments for send_ws_message()")

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
    request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, settings['attitude_frequency'])
    print(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE)
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
                    
                #send_ws_message(yaw, pitch, roll)
                send_udp_message({"type": "attitude", "values": [round(yaw, 3), round(pitch, 3), round(roll, 3)], "timestamp": int(time.time() * 1000)})
            elif message and message.get_type() == 'DISTANCE_SENSOR':
                distance_rangefinder = message.current_distance / 100.0  # Distance in meters
                #print(f"Rangefinder Distance: {distance_rangefinder} meters")
                
                
        except Exception as e:
            print(f"Error in attitude control: {e}")
            break
            
        #time.sleep(0.01)


class MedianFilter:
    def __init__(self):
        self.win_size = MEDIAN_FILTER_WIN_SIZE
        self.data_list = []
        self.id = -1

    def apply(self, xy, id):
        if self.id != id:
            self.data_list = []
            self.id = id

        self.data_list.append([xy[0], xy[1], abs(xy[0]) + abs(xy[1])])

        self.data_list = self.data_list[-self.win_size:]
        sorted_data_list = sorted(self.data_list, key=lambda x: x[2])

        return sorted_data_list[len(sorted_data_list) // 2][:2]


# Marker Detection Logic
def marker_detection():
    global marker_running, distance_rangefinder
    aruco_marker_image_1 = cv2.imread('marker-1.jpg')
    aruco_marker_image_2 = cv2.imread('marker-2.jpg')
    aruco_marker_image_3 = cv2.imread('marker-3.jpg')
    if aruco_marker_image_1 is None or aruco_marker_image_2 is None or aruco_marker_image_3 is None:
        print("Error: ArUco marker images not found.")
        return

    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
    parameters = aruco.DetectorParameters_create()

    print('Loading the pre-defined ArUco markers...')
    # Create a margin around the pre-defined marker
    margin_size = 20
    color = [255, 255, 255]  # White color for margin

    marker_id_list = []
    medianFilter = MedianFilter()

    # Verify aruco markers
    for aruco_marker_image in [aruco_marker_image_1, aruco_marker_image_2, aruco_marker_image_3]:
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
            print(f"Pre-loaded marker-{len(marker_id_list) + 1} detected, ID: {ids[0][0]}")
            marker_id_list.append(ids[0][0])

    # Continue with live video stream detection
    while True:
        # Initialize video capture
        cap = cv2.VideoCapture(VIDEO_URL)

        # Set lower resolution for better performance on RPI Zero 2W
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, PROCESSING_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, PROCESSING_HEIGHT)

        if cap.isOpened():
            cap.grab()
            ret, frame = cap.retrieve()
            
            if ret:
                break
            
        print("Error: Could not open camera. reconnecting...")
        cap.release()
        time.sleep(5)
        
    # skip buffered things
    for _ in range(30):
        cap.grab()

    frn = 0
    tracking_higher_id_count = 0
    tracking_higher_id = -1

    while marker_running:
        cap.grab()
        ret, frame = cap.retrieve()

        if not ret or frame is None:
            print("Error: Frame not retrieved.")
            #break

        frn += 1
        if frn % PROCESSING_INTERVAL != 0:
            continue

        corners, ids, rejected = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

        if ids is not None:
            # logging.info(f'ids: {ids}, corners: {corners}')
            detected_marker_id_list = [ids[i][0] for i in range(len(ids)) if ids[i][0] in marker_id_list]
            detected_corner_list = [corners[i] for i in range(len(ids)) if ids[i][0] in marker_id_list]
            i, higher_id = None, -1
            # higher id is a priority
            for j, detected_marker_id in enumerate(detected_marker_id_list):
                if detected_marker_id > higher_id:
                    i = j
                    higher_id = detected_marker_id

            if i is not None:
                detected_marker_id = detected_marker_id_list[i]
                if detected_marker_id >= tracking_higher_id:
                    tracking_higher_id = detected_marker_id
                    tracking_higher_id_count = MARKER_SWITCH_FRAME_COUNT

                if detected_marker_id == tracking_higher_id and tracking_higher_id_count > 0 or detected_marker_id < tracking_higher_id and tracking_higher_id_count <= 0:
                    if detected_marker_id < tracking_higher_id and tracking_higher_id_count <= 0:
                        tracking_higher_id = detected_marker_id
                        tracking_higher_id_count = MARKER_SWITCH_FRAME_COUNT

                    corner = detected_corner_list[i].reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = corner

                    x1, y1 = topLeft
                    x2, y2 = topRight
                    x3, y3 = bottomRight
                    x4, y4 = bottomLeft
                    center_x = (x1 + x2 + x3 + x4) / 4
                    center_y = (y1 + y2 + y3 + y4) / 4

                    image_center_x = frame.shape[1] // 2
                    image_center_y = frame.shape[0] // 2

                    angle_x = -((center_x - image_center_x) / PROCESSING_WIDTH) * FOV_X
                    angle_y = -((center_y - image_center_y) / PROCESSING_HEIGHT) * FOV_Y

                    #(angle_x, angle_y) = medianFilter.apply([angle_x, angle_y], detected_marker_id)


                    print(f"Marker Detected: {detected_marker_id}, Angular Offsets: angle_x={angle_x}, angle_y={angle_y}, Distance={distance_rangefinder}")
                    # logging.info(f"Marker Detected: {detected_marker_id}, Angular Offsets: angle_x={angle_x}, angle_y={angle_y}, Distance={distance_rangefinder}")
                    # Send marker data over the second WebSocket
                    marker_data = {
                "type": "marker",
                        "markerId": int(detected_marker_id),  # Use standard int
                        "angle_x": round(float(angle_x), 3),  # Convert NumPy float to Python float
                        "angle_y": round(float(angle_y), 3),  # Convert NumPy float to Python float
                        "distance": round(float(distance_rangefinder), 3)  # Ensure distance is a standard float
                    }
                    send_landing_target(angle_x,angle_y)
                    #marker_ws.send(json.dumps(marker_data))
                    print(f"Sent over marker: {marker_data}")
                                        # Send marker data over UDP
    #                marker_data = {
     #                   "type": "marker",
      #                  "markerId": detected_marker_id,
       #                 "angle_x": round(float(angle_x), 3),
        #                "angle_y": round(float(angle_y), 3),
         #               "timestamp": int(time.time() * 1000)
          #          }
                    send_udp_message(marker_data)

        # Return back to the previous id after MARKER_SWITCH_FRAME_COUNT frames
        #tracking_higher_id_count -= 1

        #time.sleep(0.01)

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
        time.sleep(0.001)

if __name__ == "__main__":
    main()
