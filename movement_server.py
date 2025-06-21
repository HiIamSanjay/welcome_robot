from flask import Flask, request, jsonify

app = Flask(__name__)

@app.route('/move', methods=['POST'])
def move():
    data = request.json
    direction = data.get("direction")
    
    if direction not in {"forward", "backward", "left", "right", "stop"}:
        return jsonify({"error": "Invalid direction"}), 400

    print(f"🚗 Moving: {direction.upper()}")  # Simulate motor control
    return jsonify({"status": "ok", "action": direction}), 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
