"""
Example of how to request a message in a desired interval
"""

# Disable "Bare exception" warning
# pylint: disable=W0702

import time
import math
# Import mavutil
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection("/dev/serial0",baud=921600)
# Wait a heartbeat before sending commands
master.wait_heartbeat()

def request_message_interval(message_id: int, frequency_hz: float):
    """
    Request MAVLink message in a desired frequency,
    documentation for SET_MESSAGE_INTERVAL:
        https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

    Args:
        message_id (int): MAVLink message ID
        frequency_hz (float): Desired frequency in Hz
    """
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # The MAVLink message ID
        1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
        0, 0, 0, 0, # Unused parameters
        0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    )

# Configure ATTITUDE message to be sent at 2Hz
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10)

# Get some information !
while True:
    try:
        # Receive the message
        message = master.recv_match().to_dict()
        
        # Check if the message is of type 'ATTITUDE'
        if message['mavpackettype'] == 'ATTITUDE':
            # Extract the roll, pitch, and yaw values and format them to 3 decimal places
            roll = round(math.degrees(message['roll']), 3)
            pitch = round(math.degrees(message['pitch']), 3)
            yaw = round(math.degrees(message['yaw']), 3)
            
            # Print the formatted output
            print(f"Roll: {roll}°, Pitch: {pitch}°, Yaw: {yaw}°")
    except:
        pass
    
    # Sleep for the appropriate interval to match your message request rate
    time.sleep(0.01)
