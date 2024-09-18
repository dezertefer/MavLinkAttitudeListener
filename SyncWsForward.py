import time
import math
import threading
import websocket
from pymavlink import mavutil
import json
import os
import sys

# JSON configuration file path
config_file_path = "/home/pi/MavLinkAttitudeListener/config.json"

# Default settings
settings = {
    "attitude_frequency": 10,
    "reverse_roll": False,
    "reverse_pitch": False,
    "swap_roll_pitch": False
}

# Function to load settings from the JSON file
def load_config():
    global settings
    if os.path.exists(config_file_path):
        with open(config_file_path, 'r') as file:
            settings = json.load(file)
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

# WebSocket URL
ws_url = "ws://18.234.27.121:8085"

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
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, 1e6 / frequency_hz, 0, 0, 0, 0, 0
    )

# Menu function to run in interactive mode
def menu():
    if sys.stdin.isatty():  # Only run if there is a terminal (interactive mode)
        global debug_console
        while True:
            print("\nMenu:")
            print(f"1. Change frequency (current: {settings['attitude_frequency']} Hz)")
            print(f"2. Enable/Disable debug console (current: {'on' if debug_console else 'off'})")
            print(f"3. Reverse roll (current: {'on' if settings['reverse_roll'] else 'off'})")
            print(f"4. Reverse pitch (current: {'on' if settings['reverse_pitch'] else 'off'})")
            print(f"5. Swap roll and pitch (current: {'on' if settings['swap_roll_pitch'] else 'off'})")
            print("6. Exit")

            try:
                choice = input("Enter your choice: ").strip()

                if choice.startswith("frequency"):
                    # Handle frequency change
                    pass
                # Handle other menu options...
            except EOFError:
                print("No input available, skipping menu")
                break  # Exit if running in non-interactive mode
    else:
        print("Running without interactive terminal, skipping menu.")

def main():
    ws = None

    # Reconnection loop
    while True:
        try:
            # Open WebSocket connection
            ws = websocket.WebSocket()
            ws.connect(ws_url)
            print(f"Connected to WebSocket server at {ws_url}")

            while True:
                try:
                    # Receive the MAVLink message
                    message = master.recv_match(blocking=True)

                    if message is None:
                        print("No MAVLink message received.")  # Log if no message is received
                        continue

                    print(f"Received MAVLink message: {message}")  # Log received messages

                    message = message.to_dict()

                    if message['mavpackettype'] == 'ATTITUDE':
                        print(f"Processing ATTITUDE message: {message}")  # Log specific ATTITUDE message

                        # Get raw roll, pitch, and yaw values in radians
                        roll_rad = message['roll']
                        pitch_rad = message['pitch']
                        yaw_rad = message['yaw']
                        
                        # Convert and limit roll and pitch to degrees
                        roll_deg = round(limit_angle(math.degrees(roll_rad)), 3)
                        pitch_deg = round(limit_angle(math.degrees(pitch_rad)), 3)
                        yaw_deg = round(math.degrees(yaw_rad), 3)  # Yaw can be 0 to 360 degrees

                        # Apply roll or pitch reversal if enabled
                        if settings["reverse_roll"]:
                            roll_deg = -roll_deg
                        if settings["reverse_pitch"]:
                            pitch_deg = -pitch_deg

                        # Swap roll and pitch if enabled
                        if settings["swap_roll_pitch"]:
                            roll_deg, pitch_deg = pitch_deg, roll_deg

                        # Get the current timestamp in milliseconds
                        timestamp = int(time.time() * 1000)

                        # Send via WebSocket
                        send_ws_message(ws, yaw_deg, pitch_deg, roll_deg, timestamp)
                        print(f"Sent message over WebSocket: Yaw={yaw_deg}, Pitch={pitch_deg}, Roll={roll_deg}, Timestamp={timestamp}")

                    elif message['mavpackettype'] == 'AHRS2':
                        print("AHRS2 Message:", message)

                except websocket.WebSocketConnectionClosedException as e:
                    print(f"WebSocket connection closed: {e}")
                    break  # Exit inner loop to reconnect

                except Exception as e:
                    print(f"Error: {e}")
                    break  # Exit inner loop to reconnect

        except Exception as e:
            print(f"Failed to connect to WebSocket: {e}")
            time.sleep(5)  # Wait before retrying connection

        finally:
            if ws:
                ws.close()

# Start the program
if __name__ == "__main__":
    # Only run the menu in interactive mode
    if sys.stdin.isatty():
        threading.Thread(target=menu, daemon=True).start()

    # Start the main program logic
    main()
