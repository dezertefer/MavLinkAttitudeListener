#!/usr/bin/env python3

import socket
import sys
import json

SOCKET_PATH = "/tmp/attitudeForward.sock"
CONFIG_FILE = "/home/pi/MavLinkAttitudeListener/config.json"

# Function to load and save settings
def save_config(key, value):
    try:
        with open(CONFIG_FILE, 'r') as f:
            settings = json.load(f)
        settings[key] = value
        with open(CONFIG_FILE, 'w') as f:
            json.dump(settings, f, indent=4)
        print(f"{key} updated to: {value}")
    except FileNotFoundError:
        print("Configuration file not found!")
    except Exception as e:
        print(f"Error updating {key}: {e}")

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
        save_config("ws_url", new_url)
    elif command == "reverse_pitch_on":
        save_config("reverse_pitch", True)
    elif command == "reverse_pitch_off":
        save_config("reverse_pitch", False)
    elif command == "fix_yaw":
        if len(sys.argv) < 3:
            print("Usage: attitudeForward fix_yaw <angle>")
            sys.exit(1)
        yaw_angle = float(sys.argv[2])
        save_config("fixed_yaw_angle", yaw_angle)
    elif command == "fix_yaw_off":
        save_config("fixed_yaw_angle", None)
    elif command == "reverse_yaw_on":
        save_config("reverse_yaw", True)
    elif command == "reverse_yaw_off":
        save_config("reverse_yaw", False)
    elif command == "swap_pitch_roll_on":
        save_config("swap_pitch_roll", True)
    elif command == "swap_pitch_roll_off":
        save_config("swap_pitch_roll", False)
    else:
        command_str = " ".join(sys.argv[1:])
        send_command(command_str)
