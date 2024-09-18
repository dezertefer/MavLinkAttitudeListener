
# MavLink Attitude Listener

This project allows you to listen to MAVLink messages, forward `ATTITUDE` messages over a WebSocket, and control settings dynamically through a command-line interface.

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/dezertefer/MavLinkAttitudeListener.git
   ```

2. Run the installation script:
   ```bash
   cd MavLinkAttitudeListener
   chmod +x install.sh
   ./install.sh
   ```

   This script will:
   - Install Python and dependencies.
   - Install required Python packages.
   - Set up and enable the systemd service.
   - Start the service.

## Commands

### To change the WebSocket URL:
   ```bash
   ./attitudeForward.py set_websocket_url <new_url>
   ```

### To enable/disable reverse roll:
   ```bash
   ./attitudeForward.py reverse_roll_on
   ./attitudeForward.py reverse_roll_off
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

