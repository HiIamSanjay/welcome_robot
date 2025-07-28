import requests
import time

url = "http://localhost:5000/move"

# Commands to simulate
commands = ["forward", "left", "right", "backward", "stop"]

for cmd in commands:
    print(f"Sending: {cmd}")
    res = requests.post(url, json={"direction": cmd})
    print("Response:", res.json())
    time.sleep(1)
