Welcome Robot – Gemini & Computer Vision Edition
A cross-platform Python + Flutter + Flask application that greets visitors, listens to their questions, and responds using Google's Generative AI. The system includes real-time face detection, speech recognition, AI-powered replies, Text-to-Speech, and mobile robot movement control via a joystick app.

The emotional face animations are designed to run on a dedicated Raspberry Pi Zero, receiving cues from the main control unit to change expressions.

🤖 System Architecture
This project is composed of several key components working together:

Main Control Unit (Raspberry Pi 4): This is the "brain" of the robot. It runs the main Python application that handles face detection, speech recognition, communication with the Gemini AI, and the Flask web server for movement commands.

Face Display Unit (Raspberry Pi Zero): A dedicated, lightweight Raspberry Pi that runs the Pygame-based eye_animation.py script. Its sole purpose is to display the robot's expressive face, making the interaction more engaging.

Mobile Controller (Flutter App): A cross-platform mobile app that provides a joystick for manual control and displays a live video feed from the robot's webcam.

✨ Key Features
✅ Face Detection (OpenCV): Detects human faces to initiate interaction.

✅ Speech Recognition: Captures voice commands and converts them to text.

✅ Gemini AI (Google Generative AI): Processes questions and generates intelligent, conversational replies.

✅ TTS (Text-to-Speech): Speaks replies aloud using offline (pyttsx3) or online (gTTS) engines.

✅ GUI Interface: The main application shows the webcam feed, conversation history, and TTS output.

✅ Flask Movement API: A simple web server that accepts HTTP requests to control the robot's motors.

✅ Flutter App: A cross-platform controller with a live video feed and a virtual joystick for remote operation.

🖥️ Requirements
Hardware:

Raspberry Pi 4 (or a PC/Linux machine) for the main control unit.

Raspberry Pi Zero (or another Pi) for the face display.

Webcam and Microphone connected to the main control unit.

Software:

Python 3.10 or 3.11 (avoid 3.13 due to potential library incompatibilities).

Flutter 3.16+ for building the mobile/web controller.

🛠️ Installation
Follow the steps for each component of the project.

Part 1: Main Control Unit (Raspberry Pi 4 or PC)
These steps are for the primary device running the AI and movement server.

🍓 FOR RASPBERRY PI:

System Update & Core Dependencies:

Bash

sudo apt update && sudo apt upgrade -y
sudo apt install python3-pyaudio python3-opencv ffmpeg espeak portaudio19-dev -y
Python Libraries:

Bash

pip install flask opencv-python pillow SpeechRecognition pyaudio pyttsx3 google-generativeai pygame gtts pydub
🪟 FOR WINDOWS:

Bash

pip install flask
pip install opencv-python
pip install pillow
pip install SpeechRecognition
pip install pyaudio
pip install pyttsx3
pip install google-generativeai
pip install pygame
pip install gtts
pip install pydub
📢 TTS Notes:

On Raspberry Pi, pyttsx3 uses the espeak engine for offline TTS.

On Windows, pyttsx3 uses the built-in Microsoft SAPI5 for offline TTS.

gTTS provides higher-quality voices but requires an active internet connection on any platform.

Part 2: Face Display Unit (Raspberry Pi Zero)
These steps are for the separate device that will only show the eyes.

System Update:

Bash

sudo apt update && sudo apt upgrade -y
Python Libraries: The face display only needs Pygame and NumPy.

Bash

pip install pygame numpy
Get the Code:

Create a file named eye_animation.py on the Pi Zero.

Copy the final, polished code for the Multi-Emotion Face Simulator into this file.

🔑 Set Gemini API Key
You need a Google Gemini API key to enable the AI conversation feature.

Get your key from: https://ai.google.dev/

Set the key. Choose one of the following methods:

Option A (Recommended) – Set an environment variable:

Bash

# On Linux/Raspberry Pi
export GEMINI_API_KEY="your-key-goes-here"

# On Windows
setx GEMINI_API_KEY "your-key-goes-here"
Option B – Hardcode into the Python script:
In welcome_robot_module3_gemini.py, add the line:

Python

GEMINI_API_KEY = "your-key-goes-here"
▶️ How to Run
Step 1: Run the Face Display
On your Raspberry Pi Zero, run the eye animation script:

Bash

python3 eye_animation.py
The face will appear on the screen connected to the Pi Zero, ready to receive commands to change expressions.

Step 2: Run the Main AI & Movement Server
On your Raspberry Pi 4, you have two main scripts:

For AI Interaction (welcome_robot_module3_gemini.py):

Bash

python welcome_robot_module3_gemini.py
The program will start, detect a face, and begin listening for questions. Buttons on the GUI allow for manual text input and manual voice recognition.

For Movement Control (welcome_robot_server.py):

Bash

python welcome_robot_server.py
This launches the Flask server, which is required for the Flutter controller app to work. It serves the video feed and listens for movement commands.

Step 3: Use the Flutter Controller App
Make sure welcome_robot_server.py is running on your Pi 4.

Get your Pi 4's local IP address (see instructions below).

Open the Flutter project and update the serverIp variable in lib/main.dart with the Pi 4's IP address.

Ensure your phone and the Raspberry Pi 4 are on the same Wi-Fi network.

Run the app on your device:

Bash

flutter run
⚙️ Server Endpoints (Module 5)
The welcome_robot_server.py provides the following API endpoints:

POST /move: Accepts a JSON payload to control movement. Example: {"direction": "forward"}.

GET /video_feed: Provides an MJPEG stream from the webcam for viewing in the mobile app or a web browser.

You can use test_movement_client.py to test the movement commands without needing the hardware or Flutter app.

📱 Flutter Controller App Details (Module 6)
The Flutter app provides a complete remote-control interface.

Requirements: Flutter SDK (3.16+)

Dependencies (in pubspec.yaml):

YAML

flutter_joystick: ^0.2.2
flutter_mjpeg: ^2.0.0
http: ^0.13.6
Key Feature: The flutter_mjpeg widget has a timeout of 15 seconds to prevent the app from crashing if the video stream is temporarily unavailable.

💡 Troubleshooting
pip not found: Use python3 -m pip install ... on Linux/Pi or py -m pip install ... on Windows.

PyAudio errors on Windows: You may need to install a pre-compiled wheel file from Gohlke's Python Libs page.

Connection refused: Ensure the welcome_robot_server.py script is running and that your phone and Pi are on the same network with no firewall blocking the connection.

Webcam not showing: Try changing cv2.VideoCapture(0) to cv2.VideoCapture(1) (or another index) in the Python script.

Mobile app: TimeoutException: Your video stream is not reachable. Double-check the IP address in main.dart and ensure the Flask server is running and accessible.

🌐 Getting Your Local IP Address
You will need the local IP of the machine running the Flask server to connect the Flutter app.

🪟 Windows: Run ipconfig in Command Prompt. Find the "IPv4 Address" under your active Wi-Fi or Ethernet adapter.

🍕 Mac: Run ifconfig in Terminal. Look for the en0 or en1 interface and find the inet address.

🐙 Linux/Raspberry Pi: Run hostname -I or ip a in the terminal.

📄 License
MIT License – Free for personal, educational, and academic use.

Created by Sanjay, Sager, Rishi, Adithya | Powered by Python, Flutter & Gemini AI
