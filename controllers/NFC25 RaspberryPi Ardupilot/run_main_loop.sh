#!/bin/bash

# Set the log directory
LOG_DIR="LOGS"
mkdir -p $LOG_DIR

# Activate the virtual environment
source mavlink_env/bin/activate
echo "Virtual environment activated"

while true; do
    # Generate timestamp for filename
    TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
    LOG_FILE="$LOG_DIR/mavlink_log_$TIMESTAMP.txt"
    
    # Run the main script and redirect output to log file
    echo "Starting NFC25 Autopilot script at $TIMESTAMP" | tee -a "$LOG_FILE"
    echo "----------------------------------------" | tee -a "$LOG_FILE"
    
    # Debug information
    echo "Python version:" | tee -a "$LOG_FILE"
    python --version 2>&1 | tee -a "$LOG_FILE"
    echo "Working directory: $(pwd)" | tee -a "$LOG_FILE"
    echo "Checking if main_rasp_code.py exists:" | tee -a "$LOG_FILE"
    ls -la main_rasp_code.py 2>&1 | tee -a "$LOG_FILE"
    echo "Checking required libraries:" | tee -a "$LOG_FILE"
    pip list | grep -E "pymavlink|sx126x" 2>&1 | tee -a "$LOG_FILE"
    echo "----------------------------------------" | tee -a "$LOG_FILE"
    
    # Run with explicit error handling
    echo "Attempting to run main_rasp_code.py..." | tee -a "$LOG_FILE"
    python -u main_rasp_code.py 2>&1 | tee -a "$LOG_FILE"
    EXIT_CODE=$?
    
    # Log the crash/exit with exit code
    echo "" | tee -a "$LOG_FILE"
    echo "Script terminated with exit code $EXIT_CODE at $(date +"%Y-%m-%d %H:%M:%S")" | tee -a "$LOG_FILE"
    echo "Restarting in 1 seconds..." | tee -a "$LOG_FILE"
    echo "----------------------------------------" | tee -a "$LOG_FILE"
    
    # Sleep longer to make troubleshooting easier
    sleep 1
done