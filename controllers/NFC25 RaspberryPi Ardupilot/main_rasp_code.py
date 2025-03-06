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
TELEM_PERIOD = 2          # Every 2 seconds, pause telemetry sending for ...
TELEM_PAUSE_TIME = 1     # ... 1 second

RCV_COMPL_CMD_PERIOD = 3  # Period to consider a command complete [s]
# ---------------------------------------------


# ======== NFC25 Autopilot Class ========
class NFC25Autopilot:
    def __init__(self, mav_port='/dev/serial0', mav_baud=57600, lora_port='/dev/ttyS0', lora_freq=868, lora_power=22, lora_air_speed=2400, lora_addr=0, GROUND_START_PTRN="/*", GROUND_END_PTRN="*/"):


        # MAVLink connection
        self.mav_connection = mavutil.mavlink_connection(mav_port, baud=mav_baud, autoreconnect=True)

        # Variables to store data
        self.data_lock = threading.Lock()
        self.telem_data = {
            'angle_of_attack': 0,
            'altitude': 0,
            'latitude': 0,
            'longitude': 0,
            'vx': 0,        # Ground speed in m/s - X Latitude Positive North
            'vy': 0,        # Ground speed in m/s - Y Longitude Positive East
            'vz': 0,        # Ground speed in m/s - Z Altitude Positive Down
            'v_norm': 0,    # Ground speed in m/s - Norm of the velocity vector
            'imu_raw': {'x': 0, 'y': 0, 'z': 0},
            'g_force': 0,
            'bat_voltage': 0,
            'bat_current': 0
        }
        
        # Start the MAVLink reading thread
        self.mavlink_thread = threading.Thread(target=self._readMavlinkData, daemon=True)
        self.mavlink_thread.start()

        # ======== LoRa Communication Setup ========
        # Initialize LoRa connection
        self.lora_node = sx126x.sx126x(serial_num=lora_port, freq=lora_freq, addr=lora_addr, power=lora_power, rssi=False, air_speed=lora_air_speed, relay=False)
        
        # Add a lock for LoRa operations
        self.lora_lock = threading.Lock()
        
        # Add a buffer for accumulating message fragments
        self.command_buffer = ""
        self.buffer_timeout = time.time()
        self.MAX_BUFFER_AGE = 1    # Max age of buffer in seconds
        self.last_command = ""
        self.last_complete_cmd_time = 0

        self.GROUND_START_PTRN = GROUND_START_PTRN
        self.GROUND_END_PTRN = GROUND_END_PTRN

        # Start the LoRa send and receive threads
        self.lora_send_thread = threading.Thread(target=self._sendTelemetryData, daemon=True)
        self.lora_send_thread.start()
        
        self.lora_recv_thread = threading.Thread(target=self._readLoraCommands, daemon=True)
        self.lora_recv_thread.start()

    # ======== MAVLink Communication Setup ==============================================
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
            'RC_CHANNELS_RAW': self._handleRCChannelsRaw,
            'GLOBAL_POSITION_INT': self._handleGlobalPositionInt
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
    
    def _handleGlobalPositionInt(self, msg):
        if hasattr(msg, 'lat') and hasattr(msg, 'vx') and hasattr(msg, 'vy'):
            with self.data_lock:
                self.telem_data['latitude'] = msg.lat/1e7   
                self.telem_data['longitude'] = msg.lon/1e7
                self.telem_data['vx'] = msg.vx/100          # cm/s -> m/s
                self.telem_data['vy'] = msg.vy/100          # cm/s -> m/s
                self.telem_data['vz'] = msg.vz/100          # cm/s -> m/s
                self.telem_data['v_norm'] = (msg.vx**2 + msg.vy**2 + msg.vz**2)**0.5/100  # cm/s -> m/s
    
    # =================== LoRa Telemetry Thread =========================================
    # Thread to send telemetry data via LoRa at specified frequency
    def _sendTelemetryData(self):
        last_pause = time.time()  # Track when we last paused telemetry
        while True:
            start_time = time.time()
            period = 1 / TELEM_FREQ

            # Check for periodic telemetry pause
            if time.time() - last_pause >= TELEM_PERIOD:
                print("Pausing telemetry for a while...")
                time.sleep(TELEM_PAUSE_TIME)
                last_pause = time.time()

            try:
                # First check if there might be incoming data before sending telemetry
                send_telemetry = True
                try:
                    if self.lora_node.ser.in_waiting:
                        send_telemetry = False
                        print("Skipping telemetry send - incoming data detected")
                        last_pause = time.time()  # Reset pause timer
                except Exception as e:
                    print(f"Error peeking at serial input buffer with error: {e}")

                # Send telemetry if no incoming data detected
                if send_telemetry:
                    with self.data_lock:
                        telemetry_csv = (
                            f"{time.time():.2f},"
                            f"{self.telem_data['angle_of_attack']:.2f},"
                            f"{self.telem_data['altitude']:.2f},"
                            f"{self.telem_data['g_force']:.2f},"
                            f"{self.telem_data['bat_voltage']:.2f},"
                            f"{self.telem_data['bat_current']:.2f},"
                            f"{self.telem_data['latitude']:.7f},"
                            f"{self.telem_data['longitude']:.7f},"
                            f"{self.telem_data['v_norm']:.2f}"
                        )
                        # Append command flag (1 if a complete command was received in the last 5 seconds, else 0)
                        command_flag = 1 if (time.time() - self.last_complete_cmd_time) < RCV_COMPL_CMD_PERIOD else 0
                        telemetry_csv += f",{command_flag}\n"
                    if self.lora_lock.acquire(timeout=0.1):
                        try:
                            self.lora_node.send(telemetry_csv.encode())
                            print(f"Sent telemetry: {telemetry_csv.strip()}")
                            time.sleep(0.02)  # Short delay after sending
                        finally:
                            self.lora_lock.release()
            except Exception as e:
                print(f"LoRa transmission error: {e}")

            # Calculate remaining time in this cycle
            elapsed = time.time() - start_time
            remaining = period - elapsed
            if remaining > 0:
                time.sleep(remaining)

    
    # Method to continuously check for incoming LoRa commands
    def _readLoraCommands(self):
        while True:
            try:
                # First check if there's actually data waiting before acquiring the lock
                has_data = False
                try:
                    if self.lora_node.ser.in_waiting > 0:
                        has_data = True
                except Exception:
                    print(f"Error peeking at serial input buffer with error: {e}")
                
                # Only try to acquire the lock if there's actually data to read
                if has_data and self.lora_lock.acquire(blocking=True, timeout=0.5):
                    try:
                        incoming_msg = self.lora_node.receive()
                        if incoming_msg:
                            self._bufferCommand(incoming_msg)
                    finally:
                        self.lora_lock.release()
                else:
                    # Don't sleep as long when we didn't try to acquire the lock
                    time.sleep(0.01)
                    
                # Reset buffer if it gets too old (fragmented message abandoned)
                if time.time() - self.buffer_timeout > self.MAX_BUFFER_AGE and self.command_buffer:
                    print(f"ARDUPILOT: Command buffer timed out, clearing: {self.command_buffer}")
                    self.command_buffer = ""
                    self.buffer_timeout = time.time()
                    
            except Exception as e:
                print(f"LoRa command reception error: {e}")
            
            # Shorter sleep when no data was available
            if not has_data:
                time.sleep(0.01)  # Slower checking when no data
            else:
                time.sleep(0.001)  # Fast checking when data was found
    
    # New method to buffer fragments and process when complete
    def _bufferCommand(self, fragment):
        # Update timeout since we received something
        self.buffer_timeout = time.time()
        
        # Add fragment to buffer
        self.command_buffer += fragment
        print(f"ARDUPILOT: Buffer now contains: {self.command_buffer}")
        
        # Check if we have a complete command
        if self.GROUND_START_PTRN in self.command_buffer and self.GROUND_END_PTRN in self.command_buffer:
            # Extract all complete commands from the buffer
            while self.GROUND_START_PTRN in self.command_buffer and self.GROUND_END_PTRN in self.command_buffer:
                try:
                    # Find the position of start and end patterns
                    start_pos = self.command_buffer.find(self.GROUND_START_PTRN)
                    end_pos = self.command_buffer.find(self.GROUND_END_PTRN, start_pos)
                    
                    if start_pos != -1 and end_pos != -1:
                        # Extract the command
                        cmd_start = start_pos + len(self.GROUND_START_PTRN)
                        cmd = self.command_buffer[cmd_start:end_pos]
                        

                        # Check for duplicate message
                        if cmd == self.last_command and (time.time() - self.last_complete_cmd_time) < RCV_COMPL_CMD_PERIOD:
                            # Ignore duplicate command
                            print(f"ARDUPILOT: Duplicate command received: {cmd}")
                        else:
                            # Handle the command
                            self.last_command = cmd
                            self.last_complete_cmd_time = time.time()
                            
                            # Process the command appropriately -------------
                            self._processLoraCommand(cmd)

                        # Remove the processed command from buffer
                        self.command_buffer = self.command_buffer[end_pos + len(self.GROUND_END_PTRN):]
                    else:
                        # This shouldn't happen given our check above
                        break
                except Exception as e:
                    print(f"ARDUPILOT: Error processing command: {e}")
                    self.command_buffer = ""  # Clear buffer on error
                    break
    
    # Method to process incoming LoRa commands (remove or modify as needed)
    def _processLoraCommand(self, command):
        # TODO: Process the command (implement command handling here)
        print(f"ARDUPILOT: Received complete command: {command}\n")

    # ======== Main Control Loop ========
    def run(self):
        while True:
            # print(f"Altitude: {self.telem_data['altitude']:.2f}, Angle of attack: {self.telem_data['angle_of_attack']:.2f}, G: {self.telem_data['g_force']:.2f}, Bat: {self.telem_data['bat_voltage']:.2f}V, {self.telem_data['bat_current']:.2f}A")
            time.sleep(1)  # 50 Hz update rate


if __name__ == "__main__":
    # Configure and run the autopilot class
    autopilot = NFC25Autopilot(mav_baud=MAV_BAUD, mav_port=MAV_PORT,
                               lora_port=LORA_PORT, lora_freq=LORA_FREQ, lora_power=LORA_POWER, lora_air_speed=LORA_AIR_SPEED, lora_addr=LORA_ADDR)
    autopilot.run()

    # Usefull Command
    # rm main_rasp_code.py && nano main_rasp_code.py && python main_rasp_code.py