import socket
import json
import time

# UDP server details
UDP_IP = "127.0.0.1"  # Host address
UDP_PORT = 5005       # Server port to send data from
CLIENT_IP = "127.0.0.1"  # Client address
CLIENT_PORT = 13370    # Client port to send data to

# Message to be sent
message = {
    "values": [
        -1.459,
        -29.513,
        -172.529
    ],
    "timestamp": 1729598070850,
    "accuracy": 3
}

# Convert the message to JSON
message_json = json.dumps(message)

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP socket

try:
    while True:
        # Send the message to the client
        sock.sendto(message_json.encode(), (CLIENT_IP, CLIENT_PORT))
        print(f"Sent message to {CLIENT_IP}:{CLIENT_PORT}: {message_json}")

        # Sleep for a while before sending again (optional)
        time.sleep(1)  # Send every second

except KeyboardInterrupt:
    print("Server stopped.")

finally:
    sock.close()
