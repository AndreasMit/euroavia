#!/bin/bash
# Simple time synchronization script for Raspberry Pi

if [ "$(id -u)" -ne 0 ]; then
    echo "Please run as root: sudo $0"
    exit 1
fi

echo "Restarting TimeSync..."
sudo systemctl restart systemd-timesyncd

echo "Time synchronization complete."