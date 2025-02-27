#!/usr/bin/env python3
"""
NFC25 Autonomous Flight Controller - MAVLink2 Interface
Implements all requirements from NFC25 Competition Task v1.3
"""

import threading
import time
from pymavlink import mavutil
import numpy as np
from collections import defaultdict
import statistics

# ---- MAVLink2 Interface Parameters -----------

MAV_PORT = '/dev/serial0'  # Serial port for MAVLink communication
MAV_BAUD = 57600           # Baudrate for MAVLink communication

# ---------------------------------------------


# ======== NFC25 Autopilot Class ========
class NFC25Autopilot:
    def __init__(self, mav_port='/dev/serial0', mav_baud=57600):

        # MAVLink connection
        self.mav_connection = mavutil.mavlink_connection(mav_port, baud=mav_baud)
        # Waiting for a heartbeat
        self.mav_connection.wait_heartbeat()

        # Variables to store data
        self.data_lock = threading.Lock()
        self.telem_data = {
            'angle_of_attack': 0,
            'altitude': 0,
            'imu_raw': {'x': 0, 'y': 0, 'z': 0},
            'g_force': 0,
            'bat_voltage': 0,
            'bat_current': 0
        }
        
        # Track last message timestamps for each message type
        self.last_message_times = {
            'HEARTBEAT': 0,
            'VFR_HUD': 0,
            'ATTITUDE': 0,
            'RAW_IMU': 0,
            'SYS_STATUS': 0
        }
        
        # Statistics collection
        self.frequency_data = defaultdict(list)
        self.stats_lock = threading.Lock()
        self.collecting_stats = True
        
        # Start the MAVLink reading thread
        self.mavlink_thread = threading.Thread(target=self._readMavlinkData, daemon=True)
        self.mavlink_thread.start()

    def _get_elapsed_time(self, msg_type):
        """Calculate elapsed time since last message of this type in milliseconds"""
        if self.last_message_times[msg_type] == 0:
            return 0
        return (time.time() - self.last_message_times[msg_type]) * 1000  # Convert to milliseconds

    # ======== MAVLink Communication Setup ========
    def _configure_mavlink_streams(self):
        # Disable all streams first
        self.mav_connection.mav.request_data_stream_send(self.mav_connection.target_system, self.mav_connection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 0, 0)
        # Enable all again
        # self.mav_connection.mav.request_data_stream_send(self.mav_connection.target_system, self.mav_connection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        
        mav_mesg_to_enable = [
            # Message ID, Interval
            (mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 1/1*1e6),            # 1 Hz - interval in [us]
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 1/20*1e6),            # 20 Hz - interval in [us]
            (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 1/10*1e6), # 10 Hz - interval in [us]
            (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 1/10*1e6),             # 10 Hz - interval in [us]
            (mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU, 1/20*1e6),             # 20 Hz - interval in [us]
            (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1/2*1e6)            # 2 Hz - interval in [us]
        ]

        # Enable just the necessary messages
        for mess_id, interval_us in mav_mesg_to_enable:
            self.mav_connection.mav.command_long_send(
                self.mav_connection.target_system,
                self.mav_connection.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,              # confirmation
                mess_id,        # param1: Message ID (e.g. mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE)
                interval_us,    # param2: Desired interval in microseconds (set to -1 to disable)
                0, 0, 0, 0, 0
            )

        # Small Delay after configuration
        time.sleep(1)

    # Read MAVLink data
    def _readMavlinkData(self):
        # Initialise message handlers
        self.message_handlers = {
            'HEARTBEAT': self._handleHeartbeat,
            'VFR_HUD': self._handleVfrHud,
            'ATTITUDE': self._handleAttitude,
            'RAW_IMU': self._handleRawImu,
            'SYS_STATUS': self._handleSysStatus
        }

        # Configure nescassary streams
        self._configure_mavlink_streams()

        while True:
            msg = self.mav_connection.recv_match(blocking=False)
            if msg:
                msg_type = msg.get_type()
                # Update the timestamp for this message type
                if msg_type in self.last_message_times:
                    elapsed = self._get_elapsed_time(msg_type)
                    self.last_message_times[msg_type] = time.time()
                    
                # Call the handler if one exists
                handler = self.message_handlers.get(msg_type)
                if handler:
                    handler(msg, elapsed if 'elapsed' in locals() else 0)
            time.sleep(0.01)  # Check at 100 Hz to avoid CPU overload
    
    # Message handler functions
    def _handleHeartbeat(self, msg, elapsed_ms):
        if elapsed_ms == 0: 
            elapsed_ms = -1
            
        # Calculate frequency
        freq = 1/(elapsed_ms/1000) if elapsed_ms > 0 else 0
            
        # Store frequency for statistics if within reasonable limits
        if self.collecting_stats and 0 < freq < 100:
            with self.stats_lock:
                self.frequency_data['HEARTBEAT'].append(freq)
                
        print(f"HEARTBEAT \t {freq:.2f}\n")
        # TODO: Implement lost of communication detection
        pass
    
    def _handleVfrHud(self, msg, elapsed_ms):
        if elapsed_ms == 0: 
            elapsed_ms = -1
            
        # Calculate frequency
        freq = 1/(elapsed_ms/1000) if elapsed_ms > 0 else 0
            
        # Store frequency for statistics if within reasonable limits
        if self.collecting_stats and 0 < freq < 100:
            with self.stats_lock:
                self.frequency_data['VFR_HUD'].append(freq)
                
        print(f"VFR_HUD \t {freq:.2f}\n")
        
        if hasattr(msg, 'alt'):
            with self.data_lock:
                self.telem_data['altitude'] = msg.alt

    def _handleAttitude(self, msg, elapsed_ms):
        if elapsed_ms == 0: 
            elapsed_ms = -1
            
        # Calculate frequency
        freq = 1/(elapsed_ms/1000) if elapsed_ms > 0 else 0
            
        # Store frequency for statistics if within reasonable limits
        if self.collecting_stats and 0 < freq < 100:
            with self.stats_lock:
                self.frequency_data['ATTITUDE'].append(freq)
                
        print(f"ATTITUDE \t {freq:.2f}\n")
        
        if hasattr(msg, 'pitch'):
            with self.data_lock:
                self.telem_data['angle_of_attack'] = msg.pitch

    def _handleRawImu(self, msg, elapsed_ms):
        if elapsed_ms == 0: 
            elapsed_ms = -1
            
        # Calculate frequency
        freq = 1/(elapsed_ms/1000) if elapsed_ms > 0 else 0
            
        # Store frequency for statistics if within reasonable limits
        if self.collecting_stats and 0 < freq < 100:
            with self.stats_lock:
                self.frequency_data['RAW_IMU'].append(freq)
                
        print(f"RAW_IMU \t {freq:.2f}\n")
        
        if hasattr(msg, 'xacc') and hasattr(msg, 'yacc') and hasattr(msg, 'zacc'):
            x_accel = msg.xacc/1e3
            y_accel = msg.yacc/1e3
            z_accel = msg.zacc/1e3
            g_force = (x_accel**2 + y_accel**2 + z_accel**2)**0.5
            
            with self.data_lock:
                self.telem_data['imu_raw'] = {
                    'x': x_accel,
                    'y': y_accel,
                    'z': z_accel
                }
                self.telem_data['g_force'] = g_force

    def _handleSysStatus(self, msg, elapsed_ms):
        if elapsed_ms == 0: 
            elapsed_ms = -1
            
        # Calculate frequency
        freq = 1/(elapsed_ms/1000) if elapsed_ms > 0 else 0
            
        # Store frequency for statistics if within reasonable limits
        if self.collecting_stats and 0 < freq < 100:
            with self.stats_lock:
                self.frequency_data['SYS_STATUS'].append(freq)
                
        print(f"SYS_STATUS \t {freq:.2f}\n")
        
        if hasattr(msg, 'voltage_battery') and hasattr(msg, 'current_battery'):
            with self.data_lock:
                self.telem_data['bat_voltage'] = msg.voltage_battery/1e3  # Convert to [V]
                self.telem_data['bat_current'] = msg.current_battery/1e2  # Convert to [A]
    
    def print_statistics(self):
        """Calculate and print statistics for collected frequency data"""
        print("\n\n===== MESSAGE FREQUENCY STATISTICS =====")
        print(f"Message Type\tCount\tMean (Hz)\tStd Dev (Hz)\tMin (Hz)\tMax (Hz)")
        
        with self.stats_lock:
            for msg_type, freqs in self.frequency_data.items():
                if freqs:
                    count = len(freqs)
                    mean = statistics.mean(freqs)
                    stdev = statistics.stdev(freqs) if count > 1 else 0
                    min_freq = min(freqs)
                    max_freq = max(freqs)
                    print(f"{msg_type}\t{count}\t{mean:.2f}\t{stdev:.2f}\t{min_freq:.2f}\t{max_freq:.2f}")
                else:
                    print(f"{msg_type}, 0, N/A, N/A, N/A, N/A")
        
        print("========================================\n")

    # ======== Main Control Loop ========
    def run(self, duration=None):
        """Run the autopilot for a specific duration in seconds, or indefinitely if None"""
        start_time = time.time()
        
        try:
            while True:
                # Check if we've reached the time limit
                if duration and (time.time() - start_time >= duration):
                    break
                
                # Print telemetry data including battery info
                # print(f"Altitude: {self.telem_data['altitude']:.2f}, Angle of attack: {self.telem_data['angle_of_attack']:.2f}, " +
                #       f"G: {self.telem_data['g_force']:.2f}, Bat: {self.telem_data['bat_voltage']:.2f}V, {self.telem_data['bat_current']:.2f}A")
                
                time.sleep(1/50)  # 50 Hz update rate
                
        except KeyboardInterrupt:
            print("Autopilot interrupted by user")
            
        finally:
            # Stop collecting statistics
            self.collecting_stats = False
            
            # Print statistics
            if duration:
                self.print_statistics()


if __name__ == "__main__":
    autopilot = NFC25Autopilot(mav_baud=MAV_BAUD, mav_port=MAV_PORT)
    
    # Run for 20 seconds and then print statistics
    print("Running autopilot for 20 seconds to collect statistics...")
    autopilot.run(duration=20)