#!/usr/bin/env python3
"""
NFC25 Autonomous Flight Controller - MAVLink2 Interface
Implements all requirements from NFC25 Competition Task v1.3
"""

import threading
import time
from pymavlink import mavutil
import sx126x

# ---- MAVLink2 Interface Parameters -----------

MAV_PORT = '/dev/serial0'  # Serial port for MAVLink communication
MAV_BAUD = 57600           # Baudrate for MAVLink communication

# ---------------------------------------------

# ---- LoRa Interface Parameters ---------------

LORA_PORT = '/dev/ttyUSB0'  # Serial port for LoRa communication
LORA_FREQ = 868           # Frequency for LoRa communication
LORA_POWER = 22           # Power for LoRa communication
LORA_AIR_SPEED = 2400     # Air speed for LoRa communication
LORA_ADDR = 0             # Address for LoRa communication

TELEM_FREQ = 2            # Rasp -> GCS Telemetry frequency [Hz]

# ---------------------------------------------


# ======== NFC25 Autopilot Class ========
class NFC25Autopilot:
    def __init__(self, mav_port='/dev/serial0', mav_baud=57600, lora_port='/dev/ttyS0', lora_freq=868, lora_power=22, lora_air_speed=2400, lora_addr=0):

        # MAVLink connection
        self.mav_connection = mavutil.mavlink_connection(mav_port, baud=mav_baud, autoreconnect=True)

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
        
        # Initialize LoRa connection
        self.lora_node = sx126x.sx126x(serial_num=lora_port, freq=lora_freq, addr=lora_addr, power=lora_power, rssi=True, air_speed=lora_air_speed, relay=False)
        
        # Start the MAVLink reading thread
        self.mavlink_thread = threading.Thread(target=self._readMavlinkData, daemon=True)
        self.mavlink_thread.start()

        # Start the LoRa telemetry thread
        self.lora_thread = threading.Thread(target=self._sendTelemetryData, daemon=True)
        self.lora_thread.start()

    # ======== MAVLink Communication Setup ========
    def _configure_mavlink_streams(self):
        # Disable all streams first
        self.mav_connection.mav.request_data_stream_send(self.mav_connection.target_system, self.mav_connection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 0, 0)
        # Enable all again
        # self.mav_connection.mav.request_data_stream_send(self.mav_connection.target_system, self.mav_connection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        
        mav_mesg_to_enable = [
            # Message ID, Interval
            (mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 1/1*1e6),            # 1 Hz  - interval in [us]
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 1/20*1e6),            # 20 Hz - interval in [us]
            (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 1/10*1e6), # 10 Hz - interval in [us]
            (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 1/10*1e6),             # 10 Hz - interval in [us]
            (mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU, 1/20*1e6),             # 20 Hz - interval in [us]
            (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1/2*1e6),           # 1 Hz  - interval in [us]
            (mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS_RAW, 1/5*1e6)       # 5 Hz  - interval in [us]
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


    # Read MAVLink data
    def _readMavlinkData(self):
        # Initialise message handlers
        self.message_handlers = {
            'HEARTBEAT': self._handleHeartbeat,
            'VFR_HUD': self._handleVfrHud,
            'ATTITUDE': self._handleAttitude,
            'RAW_IMU': self._handleRawImu,
            'SYS_STATUS': self._handleSysStatus,
            'RC_CHANNELS_RAW': self._handleRCChannelsRaw
        }

        # Configure nescassary streams
        self._configure_mavlink_streams()

        while True:
            msg = self.mav_connection.recv_match(blocking=False)
            if msg:
                msg_type = msg.get_type()
                # print(msg_type)
                # Call the handler if one exists
                handler = self.message_handlers.get(msg_type)
                if handler:
                    handler(msg)
            time.sleep(0.01)  # Wait a bit to avoid CPU overload
    
    # Message handler functions
    def _handleHeartbeat(self, msg):
        # TODO: Implement lost of communication detection
        pass
    
    def _handleVfrHud(self, msg):
        if hasattr(msg, 'alt'):
            with self.data_lock:
                self.telem_data['altitude'] = msg.alt

    def _handleAttitude(self, msg):
        if hasattr(msg, 'pitch'):
            with self.data_lock:
                self.telem_data['angle_of_attack'] = msg.pitch

    def _handleRawImu(self, msg):
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
    
    def _handleSysStatus(self, msg):
        if hasattr(msg, 'voltage_battery') and hasattr(msg, 'current_battery'):
            with self.data_lock:
                self.telem_data['bat_voltage'] = msg.voltage_battery/1e3 # Convert to [V]
                self.telem_data['bat_current'] = msg.current_battery/1e2 # Convert to [A]

    def _handleRCChannelsRaw(self, msg):
        if hasattr(msg, 'chan1_raw') and hasattr(msg, 'chan2_raw'):
            # TODO: Implement RC channel handling - Carefull! Test With Transmitter
            pass
    
    # ======== LoRa Telemetry Thread ========
    # Thread to send telemetry data via LoRa at specified frequency
    def _sendTelemetryData(self):
        period = 1 / TELEM_FREQ  # Desired period in seconds
        
        while True:
            start_time = time.time()
            try:
                # Format telemetry data as CSV string with unix timestamp as first field
                with self.data_lock:
                    telemetry_csv = (
                        f"{time.time():.2f},"  # Unix timestamp
                        f"{self.telem_data['angle_of_attack']:.2f},"
                        f"{self.telem_data['altitude']:.2f},"
                        f"{self.telem_data['g_force']:.2f},"
                        f"{self.telem_data['bat_voltage']:.2f},"
                        f"{self.telem_data['bat_current']:.2f},"
                        f"{self.telem_data['imu_raw']['x']:.2f},"
                        f"{self.telem_data['imu_raw']['y']:.2f},"
                        f"{self.telem_data['imu_raw']['z']:.2f}\n"
                    )
                
                # Send data via LoRa
                self.lora_node.send(telemetry_csv.encode())
                
            except Exception as e:
                print(f"LoRa transmission error: {e}")
                time.sleep(1)  # Wait a bit before retrying
            
            elapsed = time.time() - start_time
            remaining = period - elapsed
            if remaining > 0:
                time.sleep(remaining)

    # ======== Main Control Loop ========
    def run(self):
        while True:
            print(f"Altitude: {self.telem_data['altitude']:.2f}, Angle of attack: {self.telem_data['angle_of_attack']:.2f}, G: {self.telem_data['g_force']:.2f}, Bat: {self.telem_data['bat_voltage']:.2f}V, {self.telem_data['bat_current']:.2f}A")
            time.sleep(1/50)  # 50 Hz update rate


if __name__ == "__main__":
    # Configure and run the autopilot class
    autopilot = NFC25Autopilot(mav_baud=MAV_BAUD, mav_port=MAV_PORT,
                               lora_port=LORA_PORT, lora_freq=LORA_FREQ, lora_power=LORA_POWER,
                               lora_air_speed=LORA_AIR_SPEED, lora_addr=LORA_ADDR)
    autopilot.run()