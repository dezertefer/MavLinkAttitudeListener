#!/usr/bin/env python3

import socket
import sys
import json

SOCKET_PATH = "/tmp/attitudeForward.sock"
CONFIG_FILE = "/home/pi/MavLinkAttitudeListener/config.json"

# Function to load and save WebSocket URL
def save_websocket_url(new_url):
    try:
        with open(CONFIG_FILE, 'r') as f:
            settings = json.load(f)
        settings['ws_url'] = new_url
        with open(CONFIG_FILE, 'w') as f:
            json.dump(settings, f, indent=4)
        print(f"WebSocket URL updated to: {new_url}")
    except FileNotFoundError:
        print("Configuration file not found!")
    except Exception as e:
        print(f"Error updating WebSocket URL: {e}")

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
        print("Usage: attitudeForward <command> or <set_websocket_url> <new_url>")
        sys.exit(1)

    command = sys.argv[1]

    if command == "set_websocket_url":
        if len(sys.argv) < 3:
            print("Usage: attitudeForward set_websocket_url <new_url>")
            sys.exit(1)
        new_url = sys.argv[2]
        save_websocket_url(new_url)
    else:
        command_str = " ".join(sys.argv[1:])
        send_command(command_str)
