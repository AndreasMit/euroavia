import serial
import time

def main():
    port = 'COM12'      # Adjust as needed
    baud_rate = 9600    # Standard baud rate
    freq = 1.5            # [Hz]
    START_PATERN = "/*"
    END_PATERN = "*/"
    
    try:
        ser = serial.Serial(port, baud_rate, timeout=1)
        print(f"Connected to {port}")
        counter = 0
        accumulated_bytes = 0
        
        while True:
            # message = START_PATERN + f"{counter},{counter+1},{counter-1}" + END_PATERN + "\n"
            message = f"Hello, World! This is message number: {counter}\n"
            bytes_to_send = len(message)
            accumulated_bytes += bytes_to_send
            
            ser.write(message.encode('utf-8'))
            
            if accumulated_bytes >= 250:
                print(f"Sent: {message.strip()} \t Bytes: {bytes_to_send} --> Sent")
                accumulated_bytes = 0  # Reset the buffer
            else:
                print(f"Sent: {message.strip()} \t Bytes: {bytes_to_send}")
                
            counter += 1
            time.sleep(1/freq)
    
    except serial.SerialException as e:
        print(f"Error: {e}")
    
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial connection closed.")

if __name__ == "__main__":
    main()
