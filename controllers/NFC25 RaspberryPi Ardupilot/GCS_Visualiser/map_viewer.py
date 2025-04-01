"""
Standalone map viewer using pywebview.
This script is meant to be run as a separate process from the main GCS application.
"""
import sys
import os
import webview

def main():
    """Main entry point for the map viewer"""
    if len(sys.argv) < 2:
        print("Usage: map_viewer.py <html_path>")
        sys.exit(1)
    
    html_path = sys.argv[1]
    if not os.path.exists(html_path):
        print(f"HTML file not found: {html_path}")
        sys.exit(1)
    
    # Convert to URL format
    file_url = 'file://' + os.path.abspath(html_path)
    
    print(f"Opening map from: {file_url}")
    
    # Create and start webview window
    webview.create_window('Flight Map Visualizer', 
                         url=file_url,
                         width=900,
                         height=600,
                         resizable=True,
                         min_size=(800, 500),
                         text_select=True,
                         zoomable=True)
    webview.start()

if __name__ == "__main__":
    main()
