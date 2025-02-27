import threading
import time
from pymavlink import mavutil

# MAVLink connection (adjust port as needed)
connection = mavutil.mavlink_connection('/dev/serial0', baud=57600)

# Variables to store data
data_lock = threading.Lock()
data = {
    'angle_of_attack': 0,
    'altitude': 0,
    'imu_raw': {'x': 0, 'y': 0, 'z': 0},
    'g_force': 0
}

def handleVfrHud(msg):
    if hasattr(msg, 'alt'):
        with data_lock:
            data['altitude'] = msg.alt

def handleAttitude(msg):
    if hasattr(msg, 'pitch'):
        with data_lock:
            data['angle_of_attack'] = msg.pitch

def handleRawImu(msg):
    if hasattr(msg, 'xacc') and hasattr(msg, 'yacc') and hasattr(msg, 'zacc'):
        with data_lock:
            data['imu_raw'] = {
                'x': msg.xacc/1e3,
                'y': msg.yacc/1e3,
                'z': msg.zacc/1e3
            }
            data['g_force'] = (data['imu_raw']['x']**2 + data['imu_raw']['y']**2 + data['imu_raw']['z']**2)**0.5

def readMavlinkData():
    handlers = {
        'VFR_HUD': handleVfrHud,
        'ATTITUDE': handleAttitude,
        'RAW_IMU': handleRawImu
    }
    
    while True:
        msg = connection.recv_match(blocking=False)
        if msg:
            # print(f"Type: {msg.get_type()}, Content: {msg.to_dict()}")
            handler = handlers.get(msg.get_type())
            if handler:
                handler(msg)
        time.sleep(0.01)  # Check at 100 Hz to avoid CPU overload

# Start the MAVLink reading thread
mavlink_thread = threading.Thread(target=readMavlinkData, daemon=True)
mavlink_thread.start()

# Example of using the stored variables
# Disable all streams first
connection.mav.request_data_stream_send(connection.target_system, connection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 0, 0)
# Enable required streams
streams = [
    (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10),          # ATTITUDE
    (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 5),  # SYS_STATUS
    (mavutil.mavlink.MAV_DATA_STREAM_POSITION, 5),         # GLOBAL_POSITION_INT
    (mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 20),     # RAW_IMU
    (mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 10)           # VFR_HUD
    # Add more in comment and uncomment as needed
    # (mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10),     # RC_CHANNELS
    # (mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER, 10),  # ATTITUDE & POSITION CONTROLLER
]

for stream_id, rate in streams:
    connection.mav.request_data_stream_send(
        connection.target_system,
        connection.target_component,
        stream_id,
        rate,
        1 if rate > 0 else 0
    )

# enable all again
connection.mav.request_data_stream_send(connection.target_system, connection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

while True:
    with data_lock:
        print(f"Altitude: {data['altitude']:.2f}, Angle of attack: {data['angle_of_attack']:.2f}, IMU raw: {data['imu_raw']}, G: {data['g_force']:.2f}")
    time.sleep(1/50) # 50 Hz update rate
