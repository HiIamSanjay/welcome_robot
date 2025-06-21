from flask import Flask, request, jsonify, Response
import cv2

app = Flask(__name__)

# ---------- movement endpoint ----------
@app.route('/move', methods=['POST'])
def move():
    direction = request.json.get('direction', 'unknown').upper()
    print(f"🚗 Moving: {direction}")
    return jsonify({"status": "ok", "action": direction})

# ---------- live‑video endpoint ----------
@app.route('/video_feed')
def video_feed():
    def generate_frames():
        cap = cv2.VideoCapture(0)          # laptop webcam
        while True:
            success, frame = cap.read()
            if not success:
                break
            _, buffer = cv2.imencode(".jpg", frame)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' +
                   buffer.tobytes() + b'\r\n')
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    # host 0.0.0.0 lets phones on same Wi‑Fi reach your laptop
    app.run(host="0.0.0.0", port=5000, debug=True)
