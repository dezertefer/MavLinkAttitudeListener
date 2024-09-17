import time
import math
import threading
import websocket
from pymavlink import mavutil
import json
import os

# JSON configuration file path
config_file_path = "config.json"

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

# Create the connection
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
    if angle < -45:
        return -45
    elif angle > 45:
        return 45
    return angle

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
        print(f"WebSocket error: {e}")

def request_message_interval(message_id: int, frequency_hz: float):
    """Request MAVLink message at a desired frequency."""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, 1e6 / frequency_hz, 0, 0, 0, 0, 0
    )

# Function to handle the menu
def menu():
    global debug_console

    while True:
        print("\nMenu:")
        print(f"1. Change frequency (current: {settings['attitude_frequency']} Hz)")
        print(f"2. Enable/Disable debug console (type: debug on/off) (current: {'on' if debug_console else 'off'})")
        print(f"3. Reverse roll (current: {'on' if settings['reverse_roll'] else 'off'})")
        print(f"4. Reverse pitch (current: {'on' if settings['reverse_pitch'] else 'off'})")
        print(f"5. Swap roll and pitch (current: {'on' if settings['swap_roll_pitch'] else 'off'})")
        print("6. Exit")

        choice = input("Enter your choice: ").strip()

        if choice.startswith("frequency"):
            try:
                # Parse the frequency value from the input
                new_frequency = float(choice.split()[1])
                settings["attitude_frequency"] = new_frequency
                request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, new_frequency)
                save_config()
                print(f"Updated attitude message frequency to {new_frequency} Hz")
            except (IndexError, ValueError):
                print("Invalid input. Please enter a valid frequency like 'frequency 10'.")

        elif choice == "debug on":
            debug_console = True
            print("Debug console enabled. Pitch, roll, and yaw will be printed.")

        elif choice == "debug off":
            debug_console = False
            print("Debug console disabled.")

        elif choice == "reverse roll on":
            settings["reverse_roll"] = True
            save_config()
            print("Roll values will be reversed.")

        elif choice == "reverse roll off":
            settings["reverse_roll"] = False
            save_config()
            print("Roll values will no longer be reversed.")

        elif choice == "reverse pitch on":
            settings["reverse_pitch"] = True
            save_config()
            print("Pitch values will be reversed.")

        elif choice == "reverse pitch off":
            settings["reverse_pitch"] = False
            save_config()
            print("Pitch values will no longer be reversed.")

        elif choice == "swap on":
            settings["swap_roll_pitch"] = True
            save_config()
            print("Roll and pitch will be swapped.")

        elif choice == "swap off":
            settings["swap_roll_pitch"] = False
            save_config()
            print("Roll and pitch will no longer be swapped.")

        elif choice == "exit":
            print("Exiting the program...")
            break

        else:
            print("Invalid option. Please try again.")

# Start the menu in a separate thread
threading.Thread(target=menu, daemon=True).start()

def main():
    # Open WebSocket connection
    ws = websocket.WebSocket()
    ws.connect(ws_url)

    while True:
        try:
            # Receive the MAVLink message
            message = master.recv_match(blocking=True)  # blocking=True ensures waiting for the next message

            if message is None:
                continue  # Skip if no message was received

            message = message.to_dict()

            if message['mavpackettype'] == 'ATTITUDE':
                
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
                
                # Print pitch, roll, yaw in both radians and degrees if debug mode is enabled
                if debug_console:
                    print(f"Debug - Roll: {roll_rad:.3f} rad, {roll_deg:.3f} deg | "
                          f"Pitch: {pitch_rad:.3f} rad, {pitch_deg:.3f} deg | "
                          f"Yaw: {yaw_rad:.3f} rad, {yaw_deg:.3f} deg")

            elif message['mavpackettype'] == 'AHRS2':
                print("AHRS2 Message:", message)

        except Exception as e:
            print(f"Error: {e}")

# Start the program
main()
