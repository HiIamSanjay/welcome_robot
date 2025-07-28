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

✅ Dynamic AI Persona: The AI intelligently switches its personality. It acts as a friendly robot for personal chats (e.g., "how are you?") and as a professional TIST representative for college-related questions.

✅ Conversational Loop: After speaking, the robot automatically listens for a follow-up question with a 10-second timeout, allowing for a more natural, continuous conversation.

✅ Speech Recognition: Captures voice commands and converts them to text.

✅ Gemini AI: Processes questions and generates intelligent, context-aware replies based on a dynamic, multi-persona prompt.

✅ High-Quality TTS (gTTS): Uses Google's online Text-to-Speech engine for a natural, clear female voice.

✅ Themed GUI Interface: The main application features a TIST-themed UI with the college logo, brand colors, a live webcam feed, a fixed control bar, and a collapsible text input box.

✅ Flask Movement API: A simple web server that accepts HTTP requests to control the robot's motors.

✅ Flutter App: A cross-platform controller with a live video feed and a virtual joystick for remote operation.

🖥️ Requirements
Hardware:

Raspberry Pi 4 (or a PC/Linux machine) for the main control unit.

Raspberry Pi Zero (or another Pi) for the face display.

Webcam and Microphone connected to the main control unit.

Software:

Python 3.10+

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

pip install opencv-python Pillow SpeechRecognition google-generativeai gtts pygame requests
🪟 FOR WINDOWS:

Python Libraries:

Bash

pip install opencv-python Pillow SpeechRecognition google-generativeai gtts pygame requests
Part 2: Face Display Unit (Raspberry Pi Zero)
These steps are for the separate device that will only show the eyes.

System Update:

Bash

sudo apt update && sudo apt upgrade -y
Python Libraries: The face display only needs Pygame and NumPy.

Bash

pip install pygame numpy
🔑 Set Gemini API Key
You need a Google Gemini API key. Get it from: https://ai.google.dev/

Set the key by replacing "YOUR_API_KEY_HERE" inside the main Python script.

▶️ How to Run
Step 1: Run the Face Display
On your Raspberry Pi Zero, run the eye animation script:

Bash

python3 eye_animation.py
Step 2: Run the Main AI Application
On your Raspberry Pi 4 or PC, run the main AI script (e.g., main_application.py):

Bash

python main_application.py
Step 3: Run the Movement Server (Optional)
If you are using the Flutter app for movement, run the Flask server on your Pi 4:

Bash

python welcome_robot_server.py
Step 4: Use the Flutter Controller App
Ensure welcome_robot_server.py is running, update the IP address in lib/main.dart, and run the app on your phone.

⚙️ Server Endpoints
The welcome_robot_server.py provides the following API endpoints:

POST /move: Accepts a JSON payload to control movement. Example: {"direction": "forward"}.

GET /video_feed: Provides an MJPEG stream from the webcam.

💡 Troubleshooting
pip not found: Use python3 -m pip install ... on Linux/Pi or py -m pip install ... on Windows.

PyAudio errors on Windows: You may need to install a pre-compiled wheel file.

Connection refused: Ensure the Flask server is running and your phone and Pi are on the same network.

Webcam not showing: Try changing cv2.VideoCapture(0) to cv2.VideoCapture(1).

📄 License
MIT License – Free for personal, educational, and academic use.

Created by Sanjay, Sager, Rishi, Adithya | Powered by Python, Flutter & Gemini AI
