import socket

# UDP client details
UDP_IP = "127.0.0.1"  # Host address
UDP_PORT = 13370       # Port to listen on

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP socket
sock.bind((UDP_IP, UDP_PORT))  # Bind to the address and port

print(f"Listening for messages on {UDP_IP}:{UDP_PORT}...")

while True:
    data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
    print(f"Received message: {data.decode()} from {addr}")
