

**WELCOME ROBOT – GEMINI & COMPUTER VISION EDITION**

A cross-platform Python application that greets visitors, listens to their questions, and answers using Google Gemini 2.0 Flash. Combines real-time face detection, speech recognition, generative AI, and Text-to-Speech in a simple GUI.

---

**KEY FEATURES**

Face Detection (OpenCV): Detects faces and starts interaction
Speech Recognition: Captures voice and converts to text
Gemini AI: Generates replies using Google Gemini 2.0 Flash
TTS (Text-to-Speech): Speaks out replies using gTTS or pyttsx3
GUI: Shows webcam, user speech, and AI response
Flask Movement API: Receives movement commands (forward, left, right, backward, stop) through HTTP requests

---

**REQUIREMENTS**

Python 3.10 or 3.11 (Avoid 3.13)
Webcam and microphone
Works on Windows, Linux, and Raspberry Pi OS

---

**INSTALLATION**

**FOR WINDOWS:**
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

TTS Notes:
pyttsx3 uses Microsoft SAPI5 (no extra setup needed).
gTTS needs internet and works with pygame or pydub.

**FOR RASPBERRY PI:**
sudo apt update && sudo apt upgrade -y
sudo apt install python3-pyaudio python3-opencv ffmpeg espeak portaudio19-dev -y

Then:
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

TTS Notes:
pyttsx3 defaults to espeak.
gTTS works with pygame or pydub.

---

**TTS SUMMARY**
Windows: pyttsx3 + SAPI5 (offline) or gTTS (online)
Raspberry Pi: pyttsx3 + espeak (offline) or gTTS (online)

---

**SETTING GEMINI API KEY**

1. Get your API key from: [https://ai.google.dev/](https://ai.google.dev/)
2. Option 1 – Set it permanently:
   setx GEMINI\_API\_KEY "your-key"
3. Option 2 – Paste it into your Python code:
   GEMINI\_API\_KEY = "your-key"

---

**HOW TO RUN**

py welcome\_robot\_module3\_gemini.py

Steps:

1. Detect face ➜ start listening
2. Convert speech to text
3. Send to Gemini ➜ get reply
4. Speak reply using TTS

Buttons:
Submit Text – type and ask a question
Speak Now – trigger voice input

---

**MODULE 5: MOVEMENT SERVER**

Files:
movement\_server.py – Flask server that receives `/move` POST requests
test\_movement\_client.py – Simulates directional control (without hardware)

How it works:
Run the Flask server in one terminal. Then test movements by running the client script.
The server responds with action status JSON for: forward, left, right, backward, and stop.

Why:
This allows testing robot movement logic independently of hardware. Will be integrated with the Flutter app later.

---

**TROUBLESHOOTING**

pip not found: Try py -m pip install ...
pyaudio install error: Use wheel from Gohlke’s site
pygame error: Ensure audio device is free
Webcam not detected: Check connection
Gemini error: Check API key and quotas
ConnectionRefusedError: Ensure Flask server is running before client script

---

**FILE STRUCTURE**

welcome\_robot/
├─ welcome\_robot\_module1.py          → DeepSeek version
├─ welcome\_robot\_module3\_gemini.py   → Gemini + gTTS version
├─ movement\_server.py                → Flask backend for movement
├─ test\_movement\_client.py           → Test script to simulate commands
├─ README.txt                        → This file
└─ docs/
└─ demo\_face\_detection.jpg        → Optional screenshot

---

**LICENSE**

MIT License – Free for personal and educational use

---

Created by Sanjay | Powered by Python + Gemini AI


