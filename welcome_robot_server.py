from flask import Flask, request, jsonify, Response
from flask_cors import CORS
import cv2

app = Flask(__name__)
CORS(app)

@app.route('/move', methods=['POST'])
def move():
    direction = request.json.get('direction')
    print(f"🚗 Moving: {direction.upper()}")
    return jsonify({"action": direction, "status": "ok"})

@app.route('/video_feed')
def video_feed():
    def generate_frames():
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("❌ Cannot access webcam")
            return

        while True:
            success, frame = cap.read()
            if not success:
                break
            _, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n'
            )

    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, debug=True)
