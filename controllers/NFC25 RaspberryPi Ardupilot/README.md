# NFC25 Autonomous Flight Controller

## Overview
The NFC25 Autonomous Flight Controller integrates a MAVLink2 interface with LoRa telemetry. It leverages a Raspberry Pi Zero for onboard control and telemetry transmission, while the Ground Control Station (GCS) provides a GUI to visualize data.

## Project Structure
- **main_rasp_code.py**: Entry point for the Raspberry Pi Zero. Sets up MAVLink communication and manages LoRa telemetry (SX126x module).
- **GCS_Visualiser/gcs_visualiser.py**: GUI application for displaying telemetry data on the Ground Control Station.

## Setup & Usage

### Raspberry Pi Zero
1. Confirm that MAVLink and LoRa parameters (e.g., serial ports, baud rates, frequency, power settings) are correctly configured.
2. Run the autopilot:
   ```bash
   python main_rasp_code.py
   ```

### Ground Control Station (GCS)
1. Adjust serial port settings (SERIAL_PORT and BAUD_RATE) in the GCS visualiser script as needed.
2. Start the visualiser:
   ```bash
   python GCS_Visualiser/gcs_visualiser.py
   ```

## Additional Information
- **Configuration**: Detailed parameters and telemetry handling are documented within the code.
- **Testing**: Auxiliary scripts for serial and LoRa communications are available in the *Auxiliary Material* folder.
- **Telemetry Output**: Data is exported in CSV format for easy logging and processing.

## Credits
Developed by the Hermes Team for Euroavia.
