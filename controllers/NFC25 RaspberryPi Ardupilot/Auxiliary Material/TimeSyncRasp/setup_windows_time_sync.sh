#!/bin/bash
# Script to configure Raspberry Pi to use Windows PC as NTP time source

# Check if running as root
if [ "$(id -u)" -ne 0 ]; then
    echo "Please run as root: sudo $0"
    exit 1
fi

# Prompt for Windows PC IP address
echo -n "Enter Windows PC IP address: "
read WINDOWS_IP

# Backup the original ntp.conf
echo "Creating backup of NTP configuration..."
cp /etc/ntp.conf /etc/ntp.conf.backup

# Modify NTP configuration
echo "Configuring NTP to use Windows PC as time source..."
cat > /etc/ntp.conf << EOF
# NTP configuration for synchronizing with Windows PC
driftfile /var/lib/ntp/ntp.drift
statistics loopstats peerstats clockstats
filegen loopstats file loopstats type day enable
filegen peerstats file peerstats type day enable
filegen clockstats file clockstats type day enable

# Use Windows PC as time source
server ${WINDOWS_IP} prefer iburst

# Fallback to pool servers if Windows PC is unavailable
pool 0.debian.pool.ntp.org iburst
pool 1.debian.pool.ntp.org iburst
pool 2.debian.pool.ntp.org iburst
pool 3.debian.pool.ntp.org iburst

# Access control
restrict -4 default kod notrap nomodify nopeer noquery limited
restrict -6 default kod notrap nomodify nopeer noquery limited
restrict 127.0.0.1
restrict ::1
restrict ${WINDOWS_IP} nomodify notrap nopeer
EOF

# Restart NTP service
echo "Restarting NTP service..."
systemctl restart ntp

# Force immediate synchronization
echo "Forcing time synchronization..."
ntpd -gq &> /dev/null

echo "Time sync configuration complete. Current time:"
date

echo "NOTE: On your Windows PC, ensure you've enabled NTP server"
echo "1. Run Command Prompt as Administrator"
echo "2. Run: w32tm /config /syncfromflags:manual /manualpeerlist:\"time.windows.com\" /reliable:yes /update"
echo "3. Run: net stop w32time && net start w32time"