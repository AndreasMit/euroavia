"""
GCS Visualiser Utility Library
Contains helper functions, WebSocket handling, and test mode functionality for the GCS Visualiser.
"""
import math
import time
import random
import json
import asyncio
import websockets
import webbrowser
import os

# Global list of connected WebSocket clients
clients = set()
# Latest telemetry data to send to new clients
latest_telemetry = {}

# =================== WebSocket Server Functions ===================
async def websocket_handler(websocket, path):
    """Handle WebSocket connections and broadcast telemetry data."""
    # Register new client
    clients.add(websocket)
    try:
        # Send latest telemetry data to the new client
        if latest_telemetry:
            await websocket.send(json.dumps(latest_telemetry))
        
        # Keep connection alive and handle client messages if needed
        async for message in websocket:
            # Process client messages if needed
            pass
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        # Unregister client on disconnection
        clients.remove(websocket)

async def broadcast_telemetry(data):
    """Send telemetry data to all connected WebSocket clients."""
    global latest_telemetry
    latest_telemetry = data
    if clients:  # Check if there are any connected clients
        # Create message tasks for each client
        disconnected_clients = set()
        for client in clients:
            try:
                await client.send(json.dumps(data))
            except websockets.exceptions.ConnectionClosed:
                disconnected_clients.add(client)
        
        # Remove disconnected clients
        for client in disconnected_clients:
            clients.remove(client)

def start_websocket_server(port):
    """Start the WebSocket server in a background thread."""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    start_server = websockets.serve(
        websocket_handler, "localhost", port
    )
    loop.run_until_complete(start_server)
    print(f"WebSocket server started on ws://localhost:{port}")
    return loop

def launch_map_visualization(base_dir):
    """Open the map visualization in the default web browser."""
    map_path = os.path.join(base_dir, "map_visualiser.html")
    webbrowser.open('file://' + map_path)
    print(f"Opening map visualization: {map_path}")

# =================== Helper Functions ===================
def haversine_distance(lat1, lon1, lat2, lon2):
    """Calculate the great-circle distance between two points on Earth."""
    # Earth's radius in meters
    R_earth = 6371000
    
    # Convert latitudes and longitudes from degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    # Differences in coordinates
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    
    # Haversine formula components
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    # Distance between the two points in meters
    distance = R_earth * c
    return distance

# =================== Test Mode Classes and Functions ===================
class TestSerial:
    """Mock Serial class for test mode."""
    def __init__(self, home_coords):
        self.in_waiting = True
        self.last_data_time = time.time() - 0.5  # Initialize to generate data immediately
        self.test_cmd_received = "0"
        self.test_cmd_received_time = 0
        self.lat_home, self.lon_home = home_coords
    
    def readline(self):
        """Simulate reading a line from serial port."""
        # Generate new data approximately at the rate we would receive it
        current_time = time.time()
        if current_time - self.last_data_time >= 0.5:  # Simulate ~2Hz telemetry
            self.last_data_time = current_time
            self.in_waiting = True
            return self._generate_test_telemetry()
        # No data ready yet
        self.in_waiting = False
        return b""
    
    def write(self, data):
        """Simulate writing to serial port."""
        print(f"Test mode: Command sent: {data.decode('utf-8').strip()}")
        # 70% chance of successful command reception for testing both success and failure cases
        if random.random() < 0.7:
            self.test_cmd_received = "1"
            self.test_cmd_received_time = time.time()  # Record when command was received
            print("Test mode: Command received successfully")
        return len(data)

    def _generate_test_telemetry(self):
        """Generate simulated telemetry data for test mode."""
        # Create simulated data with some variance to look realistic
        timestamp = time.time()
        angle = round(random.uniform(0, 15), 1)
        altitude = round(random.uniform(100, 500), 1)
        g_force = round(random.uniform(0.9, 1.2), 2)
        voltage = round(random.uniform(11.1, 12.6), 2)
        current = round(random.uniform(0.5, 2.0), 2)
        
        # Simulated flight path - circular pattern around home coordinates
        t = (time.time() % 60) / 60 * 2 * math.pi  # Complete circle every 60 seconds
        radius = 0.001  # Roughly 100m at equator
        lat = self.lat_home + radius * math.sin(t)
        lon = self.lon_home + radius * math.cos(t)
        
        speed = round(random.uniform(5, 15), 1)
        
        # Keep command received flag as "1" for 3 seconds after receiving a command, with 70% probability
        if self.test_cmd_received == "1":
            if time.time() - self.test_cmd_received_time > 3 or random.random() > 0.7:
                self.test_cmd_received = "0"
                print("Test mode: Command received flag reset")
        
        # Format as CSV string similar to what would come from serial
        telemetry_str = f"{timestamp:.2f},{angle},{altitude},{g_force},{voltage},{current},{lat:.6f},{lon:.6f},{speed},{self.test_cmd_received}"
        return telemetry_str.encode('utf-8')
