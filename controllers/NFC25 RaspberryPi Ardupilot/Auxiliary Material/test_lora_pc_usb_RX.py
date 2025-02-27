import serial
import serial.tools.list_ports
import time
import tkinter as tk
from tkinter import ttk

def list_serial_ports():
    """ Lists serial ports available on macOS. """
    ports = serial.tools.list_ports.comports()
    available_ports = [port.device for port in ports]
    if available_ports:
        print("Available serial ports:")
        for port in available_ports:
            print(f" - {port}")
    else:
        print("No serial ports found.")
    return available_ports


class SerialReaderApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Serial Data Statistics")
        self.counter = 0
        # Serial port setup
        self.serial_port = "/dev/cu.usbserial-0001"
        self.baud_rate = 9600
        self.old_message = "Hello, World! This is message number: -1"
        self.ser = None

        if self.serial_port:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            time.sleep(2)  # Wait for the serial connection to initialize

        # Statistics
        self.total_messages = 0
        self.correct_messages = 0
        self.incorrect_messages = 0

        # GUI Elements
        self.create_widgets()

        # Start updating the serial reading
        self.update_serial_data()

    def create_widgets(self):
        # Labels for displaying statistics
        self.total_label = ttk.Label(self.root, text="Total Messages: 0", font=("Arial", 14))
        self.total_label.pack(pady=10)

        self.correct_label = ttk.Label(self.root, text="Correct Messages: 0", font=("Arial", 14))
        self.correct_label.pack(pady=10)

        self.incorrect_label = ttk.Label(self.root, text="Incorrect Messages: 0", font=("Arial", 14))
        self.incorrect_label.pack(pady=10)

        self.success_rate_label = ttk.Label(self.root, text="Success Rate: 0%", font=("Arial", 14))
        self.success_rate_label.pack(pady=10)

    def update_serial_data(self):
        if self.ser and self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            self.total_messages += 1

            # Simulated correctness check
            data_correct = self.check_data_correctness(line)

            if data_correct:
                self.correct_messages += 1
            else:
                self.incorrect_messages += 1

            self.update_labels()

        self.root.after(100, self.update_serial_data)

    def check_data_correctness(self, data: str) -> bool:
        """Dummy function to simulate data correctness check."""
        # Example condition: data is correct if it starts with 'OK'
        start_str = 'Hello, World! This is message number: '
        startsw = data.startswith(start_str)
        number = data.split(start_str)[-1]
        old_number = self.old_message.split(start_str)[-1]
        status = 1
        if(old_number == self.old_message or number == data):
            status = 0
        else:
            #
            try:
                if(int(old_number)+1!=int(number)):
                    status = 0
            except:
                    status = 0
        print(old_number,number)
        self.old_message = data
        return status

    def update_labels(self):
        self.total_label.config(text=f"Total Messages: {self.total_messages}")
        self.correct_label.config(text=f"Correct Messages: {self.correct_messages}")
        self.incorrect_label.config(text=f"Incorrect Messages: {self.incorrect_messages}")

        if self.total_messages > 0:
            success_rate = (self.correct_messages / self.total_messages) * 100
        else:
            success_rate = 0

        self.success_rate_label.config(text=f"Success Rate: {success_rate:.2f}%")

    def on_close(self):
        if self.ser:
            self.ser.close()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = SerialReaderApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()