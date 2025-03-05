#!/bin/bash
# Script to revert NTP configuration to default

# Check if running as root
if [ "$(id -u)" -ne 0 ]; then
    echo "Please run as root: sudo $0"
    exit 1
fi

# Check if backup exists
if [ -f /etc/ntp.conf.backup ]; then
    echo "Restoring NTP configuration from backup..."
    cp /etc/ntp.conf.backup /etc/ntp.conf
    
    # Restart NTP service
    echo "Restarting NTP service..."
    systemctl restart ntp
    
    echo "NTP configuration restored to original state."
else
    echo "No backup file found at /etc/ntp.conf.backup"
    echo "Cannot revert changes automatically."
fi