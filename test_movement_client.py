import requests, time

url = "http://localhost:5000/move"
commands = ["forward", "left", "right", "backward", "stop"]

for cmd in commands:
    print(f"Sending: {cmd}")
    r = requests.post(url, json={"direction": cmd})
    print("Response:", r.json())
    time.sleep(1)
