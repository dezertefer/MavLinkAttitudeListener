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
        # Prepare the data in the desired format
        data = {
            "values": [yaw, pitch, roll],
            "timestamp": timestamp,
            "accuracy": 3
        }
        
        # Convert the dictionary to a JSON string
        message = str(data).replace("'", '"')  # Ensure correct JSON format
        await websocket.send(message)
        print(f"Sent via WebSocket: {message}")

def update_request_interval():
    """Allows the user to update the attitude message frequency."""
    while True:
        try:
            # Get the new frequency from the user
            new_frequency = float(input("Enter new frequency for attitude messages: "))
            request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, new_frequency)
            print(f"Updated attitude message frequency to {new_frequency} Hz")
        except ValueError:
            print("Invalid frequency. Please enter a number.")

def request_message_interval(message_id: int, frequency_hz: float):
    """Request MAVLink message at a desired frequency."""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, 1e6 / frequency_hz, 0, 0, 0, 0, 0
    )

# Start a thread to listen for user input and update the frequency
threading.Thread(target=update_request_interval, daemon=True).start()

# Main loop to receive MAVLink messages and send over WebSocket
while True:
    try:
        # Receive the message
        message = master.recv_match().to_dict()
        
        if message['mavpackettype'] == 'ATTITUDE':
            # Convert and limit roll and pitch
            roll = limit_angle(radians_to_degrees(message['roll']))
            pitch = limit_angle(radians_to_degrees(message['pitch']))
            yaw = radians_to_degrees(message['yaw']) % 360  # Yaw can be 0 to 360 degrees
            
            # Get the current timestamp in milliseconds
            timestamp = int(time.time() * 1000)
            
            # Print the formatted output
            print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
            
            # Send via WebSocket
            asyncio.run(send_ws_message(yaw, pitch, roll, timestamp))
        
        elif message['mavpackettype'] == 'AHRS2':
            # Display the entire AHRS2 message in the console
            print("AHRS2 Message:", message)

    except Exception as e:
        print(f"Error: {e}")
    
    # Sleep for the appropriate interval to match your message request rate
    time.sleep(0.01)
