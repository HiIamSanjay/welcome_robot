**WELCOME ROBOT – GEMINI & COMPUTER VISION EDITION**

A cross-platform Python + Flutter + Flask application that greets visitors, listens to their questions, and responds using Google Gemini 2.0 Flash. Includes real-time face detection, speech recognition, AI replies, Text-to-Speech, and mobile robot movement control via a joystick app.

---

**KEY FEATURES**

✅ Face Detection (OpenCV): Detects human faces and begins interaction
✅ Speech Recognition: Captures voice and converts to text
✅ Gemini AI (Google Generative AI): Replies to questions
✅ TTS (Text-to-Speech): Speaks replies using gTTS or pyttsx3
✅ GUI Interface: Webcam feed, conversation box, and TTS output
✅ Flask Movement API: Accepts HTTP `/move` requests to control robot
✅ Flutter App: Cross-platform controller with video feed + joystick

---

**REQUIREMENTS**

* Python 3.10 or 3.11 (avoid 3.13)
* Webcam and microphone
* Windows, Linux, or Raspberry Pi OS
* Flutter 3.16+ for mobile/web controller

---

**INSTALLATION (PYTHON APP)**

🪟 **FOR WINDOWS**:

```
pip install flask
pip install opencv-python
pip install pillow
pip install SpeechRecognition
pip install pyaudio
pip install pyttsx3
pip install google-generativeai
pip install pygame
gtts
pip install pydub
```

🎙️ **TTS Notes (Windows)**:

* pyttsx3 uses Microsoft SAPI5 (offline)
* gTTS uses internet, works with pygame or pydub

🍓 **FOR RASPBERRY PI**:

```
sudo apt update && sudo apt upgrade -y
sudo apt install python3-pyaudio python3-opencv ffmpeg espeak portaudio19-dev -y
```

Then:

```
pip install flask
pip install opencv-python
pip install pillow
pip install SpeechRecognition
pip install pyaudio
pip install pyttsx3
pip install google-generativeai
pip install pygame
gtts
pip install pydub
```

📢 **TTS Notes (Pi)**:

* pyttsx3 uses espeak (offline)
* gTTS requires internet and audio output (via pygame or pydub)

---

**SET GEMINI API KEY**

1. Visit: [https://ai.google.dev/](https://ai.google.dev/)
2. Option A – Set environment variable:

   ```bash
   setx GEMINI_API_KEY "your-key"
   ```
3. Option B – Hardcode into Python:

   ```python
   GEMINI_API_KEY = "your-key"
   ```

---

**HOW TO RUN (AI INTERACTION)**

```
py welcome_robot_module3_gemini.py
```

Steps:

1. Face is detected ➔ listening starts
2. Convert speech to text
3. Send text to Gemini ➔ get reply
4. Reply spoken aloud via TTS

🔘 Buttons:

* **Submit Text** – Manual question input
* **Speak Now** – Start voice recognition manually

---

**MODULE 5: MOVEMENT SERVER**

**Files**:

* `welcome_robot_server.py` – Flask server to handle `/move` and `/video_feed`
* `test_movement_client.py` – Test directional commands (no hardware needed)

**Endpoints**:

* `/move`: Accepts `POST` with JSON direction (`forward`, `backward`, etc.)
* `/video_feed`: MJPEG webcam stream (for mobile/web viewer)

---

**MODULE 6: FLUTTER CONTROLLER APP**

Ensure that welcome_robot_server.py is running 
to do so 
py welcome_robot_server.py

Control your robot via a Flutter app with:

* Joystick UI (flutter\_joystick)
* Live camera feed (flutter\_mjpeg)
* Sends movement commands to Flask server

**Requirements**:

* Flutter SDK (3.16+)
* Dependencies:

```yaml
flutter_joystick: ^0.2.2
flutter_mjpeg: ^2.0.0
http: ^0.13.6
```

**Run on Android/Phone**:

* Update `serverIp` in `lib/main.dart` with your PC's local IP (e.g., `192.168.1.3`)
* Make sure both phone and PC are on the same Wi-Fi
* Run:

```bash
flutter run -d android
```

📌 Timeout Fix: `flutter_mjpeg` has `timeout: Duration(seconds: 15)` to avoid TimeoutException.

---

**TROUBLESHOOTING**

* `pip not found`: Use `py -m pip install ...`
* `PyAudio` errors: Use wheel from [https://www.lfd.uci.edu/\~gohlke/pythonlibs/](https://www.lfd.uci.edu/~gohlke/pythonlibs/)
* `Connection refused`: Make sure `welcome_robot_server.py` is running
* `Webcam not showing`: Try changing OpenCV `cv2.VideoCapture(0)` to another index
* `Mobile app: TimeoutException`: Ensure your video stream is reachable at the IP address

---

**GETTING YOUR LOCAL IP**

🪟 **Windows**: Run `ipconfig` in Command Prompt ➔ find IPv4 address under Wi-Fi adapter

🍕 **Mac**: Run `ifconfig` ➔ look for `en0` ➔ `inet` address (e.g., 192.168.1.x)

🐙 **Linux**: Run `hostname -I` or `ip a`

Use this IP in your Flutter app's `serverIp` value.

---

**LICENSE**

MIT License – Free for personal, educational, and academic use.

---

Created by **Sanjay** | Powered by Python, Flutter & Gemini AI
