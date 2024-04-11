"""
Code for displaying logs in telemetry pannel.

"""
import pyserial
import urllib
import random
import string
import json
import time

BAUD_RATE = 115200
SERIAL_PORT = "COM3"

ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

SENT_MISSION_CONFIGURATION = False

BASE_URL = ""
MISSION_CONFIGURATION_URL = BASE_URL + "/"
TELEMETRY_LOGS_URL = BASE_URL + "/"

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
    send_data(MISSION_CONFIGURATION_URL, ms_conf)

def handle_logs():
    line = ser.readline().decode().strip()
    value_list = map(float, line.split(","))
    
    if len(value_list) != len(KEYS):
        print("Error: Number of values and keys should be the same!")
        return
    
    metrics_data = {
        'missionID': MISSION_ID,
        'timestamp': time.time(),
        'metrics': {
            key[1][0].lower(): key[0] for key in zip(value_list, KEYS)
        }
    }

    send_data(TELEMETRY_LOGS_URL, metrics_data)



#   Event loop
while True:
    if not SENT_MISSION_CONFIGURATION:
        handle_configuration()
        SENT_MISSION_CONFIGURATION = True
    else:
        handle_logs()
    time.sleep(0.1)