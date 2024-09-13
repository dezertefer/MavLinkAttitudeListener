"""
Example of how to request a message in a desired interval
"""

# Disable "Bare exception" warning
# pylint: disable=W0702

import time
# Import mavutil
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection("/dev/serial0",baud=921600)
# Wait a heartbeat before sending commands
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))
