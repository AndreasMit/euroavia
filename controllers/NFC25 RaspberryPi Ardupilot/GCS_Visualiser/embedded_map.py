import tkinter as tk
import os
import sys
import threading
import webbrowser
import subprocess

# Store active map viewer process
_map_process = None

def createEmbeddedMap(parent, base_dir):
    """Create and return an embedded map window
    
    Args:
        parent: Parent tkinter window
        base_dir: Base directory where the HTML file is located
    
    Returns:
        None
    """
    global _map_process
    
    map_path = os.path.join(base_dir, "map_visualiser.html")
    
    # Check if map exists
    if not os.path.exists(map_path):
        print(f"Map HTML file not found: {map_path}")
        return
    
    try:
        # Ensure viewer script exists
        viewer_path = os.path.join(base_dir, "map_viewer.py")
        if not os.path.exists(viewer_path):
            createViewerScript(viewer_path)
        
        # Launch directly without intermediate window
        _map_process = subprocess.Popen([sys.executable, viewer_path, map_path])
        print("Map opened in separate window")
        
    except Exception as e:
        print(f"Error launching map: {e}")
        # Fall back to browser
        webbrowser.open('file://' + os.path.abspath(map_path))
        print("Opened map in browser as fallback")

def createViewerScript(script_path):
    """Create a standalone map viewer script that can be run in a separate process"""
    with open(script_path, 'w') as f:
        f.write("""
import sys
import webview
import os

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: map_viewer.py <html_path>")
        sys.exit(1)
    
    html_path = sys.argv[1]
    if not os.path.exists(html_path):
        print(f"HTML file not found: {html_path}")
        sys.exit(1)
    
    # Convert to URL format
    file_url = 'file://' + os.path.abspath(html_path)
    
    # Create and start webview window
    webview.create_window('Flight Map Visualizer', 
                        url=file_url,
                        width=900,
                        height=600,
                        resizable=True,
                        min_size=(800, 500),
                        text_select=True)
    webview.start()
""")

def closeAllMapWindows():
    """Close all open map windows"""
    global _map_process
    
    if _map_process:
        try:
            _map_process.terminate()
            _map_process = None
        except:
            pass
