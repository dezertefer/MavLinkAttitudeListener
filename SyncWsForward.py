import time
import math
import threading
import websocket
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection("/dev/serial0", baud=921600)
#master.serial.flushInput()
master.wait_heartbeat()
if hasattr(master, 'port') and hasattr(master.port, 'flushInput'):
    master.port.flushInput()  # Flush the serial input buffer
# WebSocket URL
ws_url = "ws://18.234.27.121:8085"

# Set default frequency for attitude messages (in Hz)
attitude_frequency = 10  # Default frequency at startup
debug_console = False  # Debug console is disabled by default

# Conversion functions
def radians_to_degrees(rad):
    return rad * 180 / math.pi

def limit_angle(angle):
    """Limits the angle to the -45 to 45 range."""
    # Convert radians to degrees if necessary
    if angle < -45:
        return -45
    elif angle > 45:
        return 45
    return angle

def send_ws_message(ws, yaw, pitch, roll, timestamp):
    """Send data over WebSocket."""
    # Prepare the data in the desired format with 3 decimal places
    data = {
        "values": [round(yaw, 3), round(pitch, 3), round(roll, 3)],
        "timestamp": timestamp,
        "accuracy": 3
    }

    try:
        # Convert the dictionary to a JSON string and send it via WebSocket
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

def main():
    # Open WebSocket connection
    ws = websocket.WebSocket()
    ws.connect(ws_url)

    while True:
        try:
            # Receive the MAVLink message
            message = master.recv_match()

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
                # Display the entire AHRS2 message in the console
                print("AHRS2 Message:", message)

        except Exception as e:
            print(f"Error: {e}")

        # Sleep based on the attitude message frequency
        time.sleep(1 / attitude_frequency)

# Start the program
main()
