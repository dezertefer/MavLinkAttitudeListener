import time
import math
import threading
import websocket
from pymavlink import mavutil
import json
import os
import sys
import socket

# Path for UNIX domain socket file
SOCKET_PATH = "/tmp/attitudeForward.sock"

# JSON configuration file path
config_file_path = "/home/pi/MavLinkAttitudeListener/config.json"

# Default settings
settings = {
    "attitude_frequency": 10,
    "reverse_roll": False,
    "reverse_pitch": False,
    "swap_roll_pitch": False,
    "fix_yaw": False,
    "fixed_yaw": None,  # Default is no fixed yaw
    "reverse_yaw": False,
    "swap_roll_yaw": False,
    "ws_url": "ws://18.234.27.121:8085"  # Default WebSocket URL
}

# Function to load settings from the JSON file
def load_config():
    global settings
    if os.path.exists(config_file_path):
        with open(config_file_path, 'r') as file:
            settings.update(json.load(file))
            print("Configuration loaded:", settings)
    else:
        print("Configuration file not found, using default settings.")

# Function to save settings to the JSON file
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

# Debug console is disabled by default
debug_console = False  

# Conversion functions
def radians_to_degrees(rad):
    return rad * 180 / math.pi

def limit_angle(angle):
    """Limits the angle to the -45 to 45 range."""
    return max(-45, min(45, angle))

def send_ws_message(ws, yaw, pitch, roll, timestamp):
    """Send data over WebSocket."""
    data = {
        "values": [round(yaw, 3), round(pitch, 3), round(roll, 3)],
        "timestamp": timestamp,
        "accuracy": 3
    }

    try:
        ws.send(str(data).replace("'", '"'))  # Ensure correct JSON format
    except websocket.WebSocketConnectionClosedException as e:
        print(f"WebSocket closed: {e}")
        raise  # Rethrow the exception to trigger reconnection

def request_message_interval(message_id: int, frequency_hz: float):
    """Request MAVLink message at a desired frequency."""
    print(f"Requesting message {message_id} at {frequency_hz} Hz")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, 1e6 / frequency_hz, 0, 0, 0, 0, 0
    )

# Function to listen for incoming commands
def command_listener():
    if os.path.exists(SOCKET_PATH):
        os.remove(SOCKET_PATH)

    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(SOCKET_PATH)
    server.listen(1)

    print("Command listener started, waiting for commands...")

    while True:
        conn, _ = server.accept()
        with conn:
            command = conn.recv(1024).decode()
            print(f"Received command: {command}")

            if command == "reverse_roll_on":
                settings["reverse_roll"] = True
                save_config()
                conn.sendall(b"Reverse roll set to ON\n")

            elif command == "reverse_roll_off":
                settings["reverse_roll"] = False
                save_config()
                conn.sendall(b"Reverse roll set to OFF\n")

            elif command == "reverse_pitch_on":
                settings["reverse_pitch"] = True
                save_config()
                conn.sendall(b"Reverse pitch set to ON\n")

            elif command == "reverse_pitch_off":
                settings["reverse_pitch"] = False
                save_config()
                conn.sendall(b"Reverse pitch set to OFF\n")

            elif command == "fix_yaw_on":
                settings["fix_yaw"] = True
                save_config()
                conn.sendall(b"Yaw fixed to 0\n")

            elif command == "fix_yaw_off":
                settings["fix_yaw"] = False
                save_config()
                conn.sendall(b"Yaw no longer fixed\n")

            elif command == "reverse_yaw_on":
                settings["reverse_yaw"] = True
                save_config()
                conn.sendall(b"Reverse yaw set to ON\n")

            elif command == "reverse_yaw_off":
                settings["reverse_yaw"] = False
                save_config()
                conn.sendall(b"Reverse yaw set to OFF\n")

            elif command == "swap_roll_yaw_on":
                settings["swap_roll_yaw"] = True
                save_config()
                conn.sendall(b"Swap roll and yaw set to ON\n")

            elif command == "swap_roll_yaw_off":
                settings["swap_roll_yaw"] = False
                save_config()
                conn.sendall(b"Swap roll and yaw set to OFF\n")

            elif command.startswith("set_frequency"):
                try:
                    frequency = float(command.split()[1])
                    settings["attitude_frequency"] = frequency
                    request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, frequency)
                    save_config()
                    conn.sendall(f"Frequency set to {frequency} Hz\n".encode())
                except (IndexError, ValueError
