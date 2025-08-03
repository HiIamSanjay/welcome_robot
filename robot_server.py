import socket
import threading
import time
from flask import Flask, request, jsonify, Response
from flask_cors import CORS
import cv2
import serial # Import the serial library for Arduino communication

# --- Serial Communication Setup ---
# This block attempts to connect to the Arduino as soon as the script starts.
try:
    # ❗ IMPORTANT: Replace 'COM3' with your Arduino's actual COM port.
    # On Linux or Mac, the port name might look like '/dev/ttyUSB0' or '/dev/tty.usbmodem...'.
    arduino_port = 'COM3'
    baud_rate = 9600
    arduino = serial.Serial(port=arduino_port, baudrate=baud_rate, timeout=.1)
    print(f"✅ Successfully connected to Arduino on {arduino_port}")
except Exception as e:
    print(f"❌ ERROR: Could not connect to Arduino on port '{arduino_port}'.")
    print(f"   Reason: {e}")
    print("   Please check the COM port, ensure the Arduino is connected, and no other program (like the Arduino IDE's Serial Monitor) is using it.")
    arduino = None # Set to None so the script doesn't crash if Arduino isn't found

# --- Flask Web App Setup ---
app = Flask(__name__)
CORS(app) # Enable Cross-Origin Resource Sharing

# --- Network Discovery Service ---
# This helps the Flutter app find the robot automatically on the network.
def get_ip_address():
    """Find the local IP address of this machine."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # This is a trick to find the primary IP; doesn't actually connect.
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1' # Fallback to localhost
    finally:
        s.close()
    return IP

def run_discovery_service():
    """Broadcasts the server's IP address over the network using UDP."""
    host_ip = get_ip_address()
    broadcast_address = '<broadcast>' # Special address to send to all devices on the network
    discovery_port = 5000 # The port the Flutter app will be listening on
    
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        print(f"📢 Discovery service started. Broadcasting IP {host_ip} on port {discovery_port}")
        
        message = f"robot_server:{host_ip}".encode('utf-8')

        while True:
            sock.sendto(message, (broadcast_address, discovery_port))
            time.sleep(2) # Broadcast every 2 seconds

# --- Flask API Routes ---

# This route handles movement commands from the Flutter app.
@app.route('/move', methods=['POST'])
def move():
    direction = request.json.get('direction')
    print(f"🚗 Command received from app: Move {direction.upper()}")
    
    # This is the "translator" part that converts app commands to Arduino commands.
    if arduino:
        command_char = ''
        if direction == 'forward':
            command_char = 'f'
        elif direction == 'backward':
            command_char = 'b'
        elif direction == 'left':
            command_char = 'l'
        elif direction == 'right':
            command_char = 'r'
        elif direction == 'stop':
            command_char = 's'
        
        # If a valid command was found, send it to the Arduino.
        if command_char:
            try:
                arduino.write(command_char.encode())
                print(f"✉️ Sent command '{command_char}' to Arduino.")
            except Exception as e:
                print(f"❌ Error writing to Arduino: {e}")

    return jsonify({"action": direction, "status": "ok"})

# This route provides the live video stream from the webcam.
@app.route('/video_feed')
def video_feed():
    """Streams the webcam feed as an MJPEG stream."""
    def generate_frames():
        cap = cv2.VideoCapture(0) # Use camera 0
        if not cap.isOpened():
            print("❌ Cannot access webcam")
            return

        while True:
            success, frame = cap.read()
            if not success:
                break
            
            # Encode the frame as a JPEG image with 80% quality.
            _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            frame_bytes = buffer.tobytes()
            
            # Yield the frame in the multipart format required for MJPEG streams.
            yield (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n'
            )
            time.sleep(0.05) # Control frame rate to reduce latency

    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# --- Main Execution Block ---
# This is what runs when you execute `python robot_server.py`.
if __name__ == "__main__":
    # Run the discovery service in a separate thread so it doesn't block the web server.
    discovery_thread = threading.Thread(target=run_discovery_service, daemon=True)
    discovery_thread.start()
    
    # Start the Flask web server, making it accessible from any device on the network.
    app.run(host='0.0.0.0', port=5000, debug=False)
