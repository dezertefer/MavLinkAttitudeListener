
# MavLink Attitude Listener

This project allows you to listen to MAVLink messages, forward `ATTITUDE` messages over a WebSocket, and control settings dynamically through a command-line interface.

## Installation
0. Install git if not installed:
   ```bash
   sudo apt-get update
   sudo apt-get install git
   ```

1. Clone the repository:
   ```bash
   git clone https://github.com/dezertefer/MavLinkAttitudeListener.git
   ```

2. Run the installation script:
   ```bash
   cd MavLinkAttitudeListener
   chmod +x install.sh
   chmod +x attitudeForward.py
   ./install.sh
   ```

   This script will:
   - Install Python and dependencies.
   - Install required Python packages.
   - Set up and enable the systemd service.
   - Start the service.

## Commands

### To change the WebSocket URL:
**IMPORTANT**: Since changing the WebSocket URL requires stopping and restarting the service,
it is imperative to perform a **sudo reboot** after making this change.
   ```bash
   ./attitudeForward.py set_websocket_url <new_url>
   ```
By default install.sh script installs everything for user with name "cdc". If you are willing to use different user name please change 
attitudeForward.py 
   ```bash
   CONFIG_FILE = "/home/cdc/MavLinkAttitudeListener/config.json"
   ```
SyncWsForward.py
   ```bash
   # JSON configuration file path
   config_file_path = "/home/cdc/MavLinkAttitudeListener/config.json"
   ```

### To enable/disable reverse roll:
   ```bash
   ./attitudeForward.py reverse_roll_on
   ./attitudeForward.py reverse_roll_off
   ```

### To enable/disable reverse pitch:
   ```bash
   ./attitudeForward.py reverse_pitch_on
   ./attitudeForward.py reverse_pitch_off
   ```

### To enable/disable yaw fix:
   ```bash
   ./attitudeForward.py fix_yaw 90
   ./attitudeForward.py fix_yaw_off
   ```

### To enable/disable pitch & roll swap:
   ```bash
   ./attitudeForward.py swap_pitch_roll_on
   ./attitudeForward.py swap_pitch_roll_off
   ```

### To enable/disable reverse yaw:
   ```bash
   ./attitudeForward.py reverse_yaw_on
   ./attitudeForward.py reverse_yaw_off
   ```

### To set the attitude message frequency:
   ```bash
   ./attitudeForward.py set_frequency <frequency>
   ```

## Service Management

The systemd service will automatically start on boot. To manually control the service:

- **Start the service**:
   ```bash
   sudo systemctl start attitudeForward.service
   ```

- **Stop the service**:
   ```bash
   sudo systemctl stop attitudeForward.service
   ```

- **Check the service status**:
   ```bash
   sudo systemctl status attitudeForward.service
   ```

## Debugging

Each step of the installation process is output to the console. If you encounter an issue, check the output for errors, or review logs:
   ```bash
   sudo journalctl -u attitudeForward.service -e
   ```

## Uninstallation

To remove the service:
   ```bash
   sudo systemctl stop attitudeForward.service
   sudo systemctl disable attitudeForward.service
   sudo rm /etc/systemd/system/attitudeForward.service
   sudo systemctl daemon-reload
   ```

## Possible issues

Python socket doesn't have all read write permissions:
   ```bash
   cdc@devpi:~/MavLinkAttitudeListener $ ./attitudeForward.py set_frequency 100
   Error sending command: [Errno 111] Connection refused
   ```
In this case this command fixes the issue, occurs when service installed/uninstalled multiple times
   ```bash
   cdc@devpi:~/MavLinkAttitudeListener $ sudo chmod 777 /tmp/attitudeForward.sock
   cdc@devpi:~/MavLinkAttitudeListener $ sudo systemctl restart attitudeForward.service
   ```


