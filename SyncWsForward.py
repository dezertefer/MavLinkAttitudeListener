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
config_file_path = "/home/cdc/MavLinkAttitudeListener/config.json" 

# Default settings
settings = {
    "attitude_frequency": 10,
    "reverse_roll": False,
    "reverse_pitch": False,
    "swap_pitch_roll": False,
    "reverse_yaw": False,
    "fixed_yaw_angle": None,
    "ws_url": "ws://18.234.27.121:8085"  # Default WebSocket URL
}

# Function to load settings from the JSON file
def load_config():
    global settings
    if os.path.exists(config_file_path):
        with open(config_file_path, 'r') as file:
            settings.update(json.load(file))
            print("Configuration reloaded:", settings)
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

# Socket creation function
def create_socket():
    # Remove the socket file if it already exists
    if os.path.exists(SOCKET_PATH):
        os.remove(SOCKET_PATH)

    # Create a UNIX socket
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

    # Bind the socket to the path
    try:
        server.bind(SOCKET_PATH)
        server.listen(1)  # Listen for incoming connections
        print(f"Socket created and listening at {SOCKET_PATH}")
    except socket.error as e:
        print(f"Socket binding failed: {e}")
        return None
    return server

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
    
    print(f"Sending WebSocket message: Yaw={yaw}, Pitch={pitch}, Roll={roll}")  # Debugging message

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

# Main function
def main():
    ws = None
    server_socket = create_socket()  # Create the socket
    if server_socket is None:
        return  # Exit if socket creation failed

    # Load WebSocket URL from settings
    ws_url = settings.get("ws_url", "ws://18.234.27.121:8085")
    print(f"Connecting to WebSocket server at {ws_url}")

    # Request ATTITUDE messages at the specified frequency
    request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, settings['attitude_frequency'])

    # Reconnection loop
    while True:
        try:
            # Open WebSocket connection
            ws = websocket.WebSocket()
            ws.connect(ws_url)
            print(f"Connected to WebSocket server at {ws_url}")

            while True:
                load_config()  # Reload the configuration dynamically

                try:
                    # Receive the MAVLink message
                    message = master.recv_match(blocking=True)

                    if message is None:
                        continue

                    message = message.to_dict()

                    if message['mavpackettype'] == 'ATTITUDE':
                        roll_rad = message['roll']
                        pitch_rad = message['pitch']
                        yaw_rad = message['yaw']

                        # Apply any settings: reverse, swap, fixed yaw, etc.
                        roll_deg = round(limit_angle(math.degrees(roll_rad)), 3)
                        pitch_deg = round(limit_angle(math.degrees(pitch_rad)), 3)
                        yaw_deg = round(math.degrees(yaw_rad), 3)

                        # Reverse options
                        if settings["reverse_roll"]:
                            print("Reversing roll")
                            roll_deg = -roll_deg
                        if settings["reverse_pitch"]:
                            print("Reversing pitch")
                            pitch_deg = -pitch_deg
                        if settings["reverse_yaw"]:
                            print("Reversing yaw")
                            yaw_deg = -yaw_deg

                        # Fix yaw if applicable
                        if settings["fixed_yaw_angle"] is not None:
                            print(f"Fixing yaw to {settings['fixed_yaw_angle']}")
                            yaw_deg = settings["fixed_yaw_angle"]

                        # Swap pitch and roll if enabled
                        if settings["swap_pitch_roll"]:
                            print("Swapping pitch and roll")
                            roll_deg, pitch_deg = pitch_deg, roll_deg

                        timestamp = int(time.time() * 1000)
                        send_ws_message(ws, yaw_deg, pitch_deg, roll_deg, timestamp)

                except websocket.WebSocketConnectionClosedException as e:
                    print(f"WebSocket connection closed: {e}")
                    break  # Exit inner loop to reconnect

                except Exception as e:
                    print(f"Error: {e}")
                    break  # Exit inner loop to reconnect

        except Exception as e:
            print(f"Failed to connect to WebSocket: {e}")
            time.sleep(5)

        finally:
            if ws:
                ws.close()
            server_socket.close()  # Close the socket when done

if __name__ == "__main__":
    main()
