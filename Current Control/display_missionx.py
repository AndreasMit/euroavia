"""
Code for displaying logs in telemetry pannel.

"""
import serial
import urllib.request
import random
import string
import json
import time

BAUD_RATE = 115200
SERIAL_PORT = "COM9"

ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

SENT_MISSION_CONFIGURATION = False

BASE_URL = "http://localhost:8000/api"
MISSION_CONFIGURATION_URL = BASE_URL + "/missions/register"
TELEMETRY_LOGS_URL = BASE_URL + "/telemetry/add-log"

KEYS = [("time", "number", ""), 
        ("throttle", "line", ""), 
        ("frequency", "line", "Hz"), 
        ("control_enabled", "number", "")
        ]

MISSION_ID = None

#   for the missionID
def generate_random_char():
    characters = string.ascii_letters
    random_string = ''.join(random.choice(characters) for _ in range(7))    
    return random_string.upper()

# returns success, dictionary

def send_data(url, data):
    json_data = json.dumps(data).encode('utf-8')
    
    headers = {'Content-Type': 'application/json'}
    try:
        # Make the request
        req = urllib.request.Request(url, data=json_data, headers=headers, method='POST')
        with urllib.request.urlopen(req) as response:
            # Read the response
            response_data = response.read().decode('utf-8')
            print(response_data)
    except urllib.error.URLError as e:
        print("Error:", e)
    pass


def handle_configuration():
    global MISSION_ID
    MISSION_ID = generate_random_char()
    print("MISSION ID:", MISSION_ID)
    ms_conf = {
        'missionID': MISSION_ID,
        'metricsConfiguration': {
            key[0].lower(): {
                'units': key[2].lower(),
                'representation': key[1].lower(),
            } for key in KEYS
        }
    }
    # print(ms_conf)
    send_data(MISSION_CONFIGURATION_URL, ms_conf)

def validate_list(lst):
    # Check if the length of the list is 4
    if len(lst) != len(KEYS):
        return False
    
    # Check if each element in the list is an integer represented as a string
    for item in lst:
        # Check if the item is a string
        if not isinstance(item, str):
            return False
        # Try converting the string to an integer
        try:
            float(item)
        except ValueError:
            # If conversion fails, it's not an integer represented as a string
            return False
    return True

def handle_logs():
    line = ser.readline()
    print(line)
    line_list = line.decode().strip().split(",")
    
    if not validate_list(line_list):
        return False

    value_list = list(map(float, line_list))
    
    if len(value_list) != len(KEYS):
        print("Error: Number of values and keys should be the same!")
        return
    
    metrics_data = {
        'missionID': MISSION_ID,
        'timestamp': time.time_ns() // 1_000_000,
        'metrics': {
            key[1][0].lower(): key[0] for key in zip(value_list, KEYS)
        }
    }
    # print(metrics_data)
    send_data(TELEMETRY_LOGS_URL, metrics_data)



#   Event loop
while True:
    if not SENT_MISSION_CONFIGURATION:
        handle_configuration()
        SENT_MISSION_CONFIGURATION = True
    else:
        handle_logs()
    time.sleep(0.1)