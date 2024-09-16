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

# Set default frequencies (in Hz)
attitude_frequency = 10  # Default frequency for ATTITUDE messages
gps_frequency = 1        # Default frequency for GPS messages

# Debug and GPS console flags
debug_console = False  # Debug console is disabled by default
gps_console = False    # GPS console is disabled by default

# Conversion functions
def radians_to_degrees(rad):
    """Convert radians to degrees."""
    return rad * 180 / math.pi

def limit_angle(angle):
    """Limits the angle to the -45 to 45 range."""
    if angle < -45:
        return -45
    elif angle > 45:
        return 45
    return angle

async def send_ws_message(websocket, yaw, pitch, roll, timestamp):
    """Send data over WebSocket."""
    # Prepare the data in the desired format with 3 decimal places
    data = {
        "values": [round(yaw, 3), round(pitch, 3), round(roll, 3)],
        "timestamp": timestamp,
        "accuracy": 3
    }
    
    # Convert the dictionary to a JSON string and send it via WebSocket
    try:
        await websocket.send(str(data).replace("'", '"'))  # Ensure correct JSON format
    except websockets.exceptions.ConnectionClosedError as e:
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
    global attitude_frequency, gps_frequency, debug_console, gps_console

    while True:
        print("\nMenu:")
        print("1. Change frequency (type: frequency <value>)")
        print("2. Enable/Disable debug console (type: debug on/off)")
        print("3. Enable/Disable GPS console (type: gps on/off)")
        print("4. Exit")

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

        elif choice == "gps on":
            gps_console = True
            request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, gps_frequency)
            print("GPS console enabled. Latitude, longitude, and altitude will be printed.")

        elif choice == "gps off":
            gps_console = False
            # Optionally, you can stop requesting GPS messages
            request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 0)  # Stop GPS messages
            print("GPS console disabled.")

        elif choice == "exit":
            print("Exiting the program...")
            break

        else:
            print("Invalid option. Please try again.")

# Start the menu in a separate thread
threading.Thread(target=menu, daemon=True).start()

async def main():
    # Open WebSocket connection
    async with websockets.connect(ws_url) as websocket:
        while True:
            try:
                # Receive the message
                message = master.recv_match()

                if message is None:
                    continue  # Skip if no message was received

                message = message.to_dict()

                if message['mavpackettype'] == 'ATTITUDE':
                    # Get raw roll, pitch, and yaw values in radians
                    roll_rad = message['roll']
                    pitch_rad = message['pitch']
                    yaw_rad = message['yaw']
                    
                    # Convert to degrees and limit
                    roll_deg = limit_angle(radians_to_degrees(roll_rad))
                    pitch_deg = limit_angle(radians_to_degrees(pitch_rad))
                    yaw_deg = radians_to_degrees(yaw_rad) % 360  # Yaw can be 0 to 360 degrees
                    
                    # Get the current timestamp in milliseconds
                    timestamp = int(time.time() * 1000)
                    
                    # Send via WebSocket
                    await send_ws_message(websocket, yaw_deg, pitch_deg, roll_deg, timestamp)
                    
                    # Print pitch, roll, yaw in both radians and degrees if debug mode is enabled
                    if debug_console:
                        print(f"Debug - Roll: {roll_rad:.3f} rad, {roll_deg:.3f} deg | "
                              f"Pitch: {pitch_rad:.3f} rad, {pitch_deg:.3f} deg | "
                              f"Yaw: {yaw_rad:.3f} rad, {yaw_deg:.3f} deg")

                elif message['mavpackettype'] == 'GLOBAL_POSITION_INT' and gps_console:
                    # Extract latitude, longitude, and altitude
                    lat = message['lat'] / 1e7  # Convert from degrees * 1e7 to degrees
                    lon = message['lon'] / 1e7
                    alt = message['alt'] / 1000  # Convert from millimeters to meters

                    # Print GPS data
                    print(f"GPS Data - Latitude: {lat:.7f}, Longitude: {lon:.7f}, Altitude: {alt:.2f} m")

            except Exception as e:
                print(f"Error: {e}")

            # Sleep based on the highest frequency to ensure messages are processed timely
            await asyncio.sleep(0.01)

# Start the asyncio event loop
asyncio.run(main())
