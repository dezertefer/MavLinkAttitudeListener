import socket
import sys

SOCKET_PATH = "/tmp/attitudeForward.sock"

def send_command(command):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        try:
            client.connect(SOCKET_PATH)
            client.sendall(command.encode())
            response = client.recv(1024).decode()
            print(response)
        except FileNotFoundError:
            print("Service is not running or socket is not available.")
        except Exception as e:
            print(f"Error sending command: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: attitudeForward <command>")
        sys.exit(1)

    command = " ".join(sys.argv[1:])
    send_command(command)
