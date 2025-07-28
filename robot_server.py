import socket
import threading
import time
from flask import Flask, request, jsonify, Response
from flask_cors import CORS
import cv2

# --- App Setup ---
app = Flask(__name__)
CORS(app)

# --- Discovery Service ---
def get_ip_address():
    """Find the local IP address of the machine."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Doesn't have to be reachable
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP

def run_discovery_service():
    """Broadcasts the server's presence on the network."""
    host_ip = get_ip_address()
    # Use a broadcast address and a specific port
    broadcast_address = '<broadcast>' 
    discovery_port = 5000 # Port for discovery broadcast
    
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        print(f"📢 Discovery service started. Broadcasting IP {host_ip} on port {discovery_port}")
        
        # Message format: "robot_server:<ip>"
        message = f"robot_server:{host_ip}".encode('utf-8')

        while True:
            sock.sendto(message, (broadcast_address, discovery_port))
            time.sleep(2) # Broadcast every 2 seconds

# --- Flask Routes ---
@app.route('/move', methods=['POST'])
def move():
    direction = request.json.get('direction')
    # This is where you would add your robot's motor control logic
    print(f"🚗 Command received: Move {direction.upper()}")
    return jsonify({"action": direction, "status": "ok"})

@app.route('/video_feed')
def video_feed():
    """Streams the webcam feed."""
    def generate_frames():
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("❌ Cannot access webcam")
            return

        while True:
            success, frame = cap.read()
            if not success:
                break
            
            # Encode frame as JPEG
            _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            frame_bytes = buffer.tobytes()
            
            # Yield the frame in the multipart format
            yield (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n'
            )
            time.sleep(0.05) # Control frame rate to reduce latency

    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# --- Main Execution ---
if __name__ == "__main__":
    # Run discovery service in a separate thread
    discovery_thread = threading.Thread(target=run_discovery_service, daemon=True)
    discovery_thread.start()
    
    # Run Flask app
    app.run(host='0.0.0.0', port=5000, debug=False)