import socket

# The same port the server is broadcasting on
DISCOVERY_PORT = 5000

# Listen on all available network interfaces
HOST = '0.0.0.0'

print(f"--- UDP Listener Started on Port {DISCOVERY_PORT} ---")
print("Waiting for a broadcast message from the robot server...")

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((HOST, DISCOVERY_PORT))

    while True:
        data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
        message = data.decode('utf-8')
        print(f"\nSUCCESS! Received message: '{message}' from address: {addr}")