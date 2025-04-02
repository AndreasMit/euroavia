#!/bin/bash

# This script finds and terminates NFC25 Autopilot processes
# Modified to terminate the watchdog script first, then the main process

# Find all python processes running main_rasp_code.py
find_processes() {
    ps aux | grep "python.*main_rasp_code\.py" | grep -v grep
}

# Find watchdog script process
find_watchdog() {
    ps aux | grep "run_main_loop\.sh" | grep -v grep
}

# Main function
main() {
    echo "Searching for NFC25 Autopilot processes..."

    # Get matching processes
    PROCESSES=$(find_processes)
    WATCHDOG=$(find_watchdog)
    
    if [ -z "$PROCESSES" ] && [ -z "$WATCHDOG" ]; then
        echo "No NFC25 Autopilot processes or watchdog scripts found."
        exit 0
    fi

    # Display found processes
    echo "Found the following processes:"
    
    if [ ! -z "$WATCHDOG" ]; then
        echo -e "\nWatchdog script:"
        echo "$WATCHDOG"
    fi
    
    if [ ! -z "$PROCESSES" ]; then
        echo -e "\nMain processes:"
        echo "$PROCESSES"
    fi
    echo ""

    # Ask if user wants to kill these processes
    read -p "Do you want to kill these processes? (y/n): " response

    if [[ "$response" =~ ^[Yy] ]]; then
        # First terminate the watchdog script if it exists
        if [ ! -z "$WATCHDOG" ]; then
            watchdog_pid=$(echo "$WATCHDOG" | awk '{print $2}')
            echo "Terminating watchdog script (PID: $watchdog_pid) first..."
            kill -15 $watchdog_pid
            
            # Wait and check if it's still running
            sleep 1
            if ps -p $watchdog_pid > /dev/null 2>&1; then
                echo "Watchdog script still running, using SIGKILL..."
                kill -9 $watchdog_pid
                sleep 1
            fi
            echo "Watchdog script terminated."
        fi
        
        # Now terminate the main processes
        if [ ! -z "$PROCESSES" ]; then
            pids=$(echo "$PROCESSES" | awk '{print $2}')
            
            for pid in $pids; do
                echo "Terminating process $pid..."
                kill -15 $pid  # Try SIGTERM first
                
                # Wait and check if it's still running
                sleep 1
                if ps -p $pid > /dev/null 2>&1; then
                    echo "Process $pid still running, using SIGKILL..."
                    kill -9 $pid
                    sleep 1
                fi
            done
            echo "All main processes have been terminated."
        fi
        
        echo "Termination complete."
    else
        echo "No processes were killed."
    fi
}

# Run the main function
main