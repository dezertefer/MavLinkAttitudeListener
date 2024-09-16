import time
import math
import asyncio
import websockets
import threading
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection("/dev/serial0", baud=921600)
master.wait_heartbeat()

# WebSocket URL
ws_url = "ws://3.91.74.146:8485"

# Set default frequency for attitude messages (in Hz)
attitude_frequency = 10  # Default frequency at startup
debug_console = False  # Debug console is disabled by default

# Conversion functions
def radians_to_degrees(rad):
    return rad * 180 / math.pi

def limit_angle(angle, min_value=-45, max_value=45):
    """Limits the angle to the -45 to 45 range, converts -45 to 315 degrees."""
    
    if 0 <= angle < 45:
        return angle
    elif 315 <= angle < 360:
        return -(360 - angle)
    elif 180 < angle < 315:
        return -45
    elif 45 < angle < 180:
        return 45

    return max_value if angle > max_value else min_value

async def send_ws_message(yaw, pitch, roll, timestamp):
    async with websockets.connect(ws_url) as websocket:
        # Prepare the data in the desired format with 3 decimal places
        data = {
            "values": [round(yaw, 3), round(pitch, 3), round(roll, 3)],
            "timestamp": timestamp,
            "accuracy": 3
        }
        
        # Convert the dictionary to a JSON string
        message = str(data).replace("'", '"')  # Ensure correct JSON format
        await websocket.send(message)

def request_message_interval(message_id: int, frequency_hz: float):
    """Request MAVLink message at a desired frequency."""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, 1e6 / frequency_hz, 0, 0, 0, 0, 0
    )

# Function to handle the menu
def menu():
    global attitude_frequency, debug_console

    while True:
        print("\nMenu:")
        print("1. Change frequency (type: frequency <value>)")
        print("2. Enable/Disable debug console (type: debug on/off)")
        print("3. Exit")

        choice = input("Enter your choice: ").strip()

        if choice.startswith("frequency"):
            try:
                # Parse the frequency value from the input
                new_frequency = float(choice.split()[1])
                attitude_frequency = new_frequency
                request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, new_frequency)
                print(f"Updated attitude message frequency to {new_frequency} Hz")
            except (IndexError, ValueError):
                print("Invalid input. Please enter a valid frequency like 'frequency 10'.")

        elif choice == "debug on":
            debug_console = True
            print("Debug console enabled. Pitch, roll, and yaw will be printed.")

        elif choice == "debug off":
            debug_console = False
            print("Debug console disabled.")

        elif choice == "exit":
            print("Exiting the program...")
            break

        else:
            print("Invalid option. Please try again.")

# Start the menu in a separate thread
threading.Thread(target=menu, daemon=True).start()

# Main loop to receive MAVLink messages and send over WebSocket
while True:
    try:
        # Receive the message
        message = master.recv_match()

        if message is None:
            continue  # Skip if no message was received

        message = message.to_dict()

        if message['mavpackettype'] == 'ATTITUDE':
            # Convert and limit roll and pitch, rounding to 3 decimal places
            roll = round(limit_angle(radians_to_degrees(message['roll'])), 3)
            pitch = round(limit_angle(radians_to_degrees(message['pitch'])), 3)
            yaw = round(radians_to_degrees(message['yaw']) % 360, 3)  # Yaw can be 0 to 360 degrees
            
            # Get the current timestamp in milliseconds
            timestamp = int(time.time() * 1000)
            
            # Send via WebSocket
            asyncio.run(send_ws_message(yaw, pitch, roll, timestamp))
            
            # Print pitch, roll, yaw if debug mode is enabled
            if debug_console:
                print(f"Debug - Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")

        elif message['mavpackettype'] == 'AHRS2':
            # Display the entire AHRS2 message in the console
            print("AHRS2 Message:", message)

    except Exception:
        pass  # Silently ignore errors
    
    # Sleep based on the attitude message frequency
    time.sleep(1 / attitude_frequency)
