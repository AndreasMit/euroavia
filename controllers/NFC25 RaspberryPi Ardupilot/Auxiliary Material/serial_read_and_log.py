import serial
import datetime
import time

LOG_TO_FILE = False
JUST_PRINT_COMS = False

def printCOMs():
    # print available COM ports
    import serial.tools.list_ports
    ports = serial.tools.list_ports.comports()
    for port, desc, hwid in sorted(ports):
        print(f"{port}: {desc} [{hwid}]")
    

def main():
    # Configure serial port (adjust the COM port and baud rate as needed)
    ser = serial.Serial(port='COM12', baudrate=9600, timeout=1)
    
    # Create a filename based on the session date and time
    session_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"session_{session_time}.txt"

    print(f"Logging serial data to {filename}")
    
    # Track the time of the last message
    last_msg_time = time.time()

    try:
        while True:
            # Read available data from the serial port
            line = ser.readline().decode('utf-8', errors='replace').strip()
            if line:
                current_time = time.time()
                elapsed_time = current_time - last_msg_time
                freq = 1.0 / elapsed_time if elapsed_time > 0 else 0
                
                if LOG_TO_FILE:
                    # Append the received string to the file
                    with open(filename, "a", encoding="utf-8") as file:
                        file.write(line + "\n")
                print(f"Received [{freq:.2f} Hz]: {line}")
                
                # Update the last message time
                last_msg_time = current_time
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ser.close()

if __name__ == "__main__":
    if JUST_PRINT_COMS:
        printCOMs()
    else:
        main()
