#!/bin/bash

echo "Starting MavLinkAttitudeListener installation..."

# Update system and install Python
echo "Updating system and installing Python3, pip, and git..."
sudo apt-get update
sudo apt-get install -y python3 python3-pip git || { echo "Failed to install Python3, pip, or git"; exit 1; }
sudo apt install python3-opencv || { echo "Failed to install Python3, pip, or git"; exit 1; }

# Install required Python packages
echo "Installing Python dependencies..."
sudo pip3 install pymavlink websocket-client pyserial || { echo "Failed to install Python dependencies"; exit 1; }

python3 generate_aruco_marker.py --ids 500 501 502
# Detect the current user
CURRENT_USER=$(whoami)

# Set up the systemd service
echo "Setting up the systemd service..."

SERVICE_FILE=/etc/systemd/system/attitudeForward.service

sudo bash -c "cat > $SERVICE_FILE" << EOL
[Unit]
Description=MavLink Attitude Forward Service
After=mavlink-router.service
Wants=mavlink-router.service

[Service]
WorkingDirectory=$(pwd)
ExecStart=/usr/bin/python3 $(pwd)/SyncWsForward.py
User=$CURRENT_USER
Restart=always
Environment="PYTHONPATH=/usr/local/bin:/usr/bin:/bin"
ExecStartPre=/bin/mkdir -p /tmp

[Install]
WantedBy=multi-user.target
EOL

# Reload the systemd manager configuration
sudo systemctl daemon-reload

# Enable the service to start on boot
echo "Enabling attitudeForward service..."
sudo systemctl enable attitudeForward.service || { echo "Failed to enable service"; exit 1; }

# Start the service
echo "Starting attitudeForward service..."
sudo systemctl start attitudeForward.service || { echo "Failed to start service"; exit 1; }

echo "Installation completed!"
