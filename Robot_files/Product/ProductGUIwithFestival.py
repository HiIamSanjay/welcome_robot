# Product GUI v1.5 - Switched to robust Festival pipe-based TTS.
import cv2
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
import speech_recognition as sr
import google.generativeai as genai
import subprocess
import time
import webbrowser
import requests
import socket
from enum import Enum
import io
import os
import sys
from ctypes import CFUNCTYPE, cdll, c_char_p, c_int
import face_recognition # Still needed for face location
import numpy as np
from deepface import DeepFace

# --- Function to Suppress ALSA Error Messages ---
def py_error_handler(filename, line, function, err, fmt):
    pass
ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)
def suppress_alsa_errors():
    try:
        asound = cdll.LoadLibrary('libasound.so.2')
        asound.snd_lib_error_set_handler(c_error_handler)
    except (OSError, AttributeError):
        pass

# --- Configuration Constants ---
GEMINI_API_KEY = "AIzaSyAh26HcQ76_kc4Spy8XjXJskwSCMNiFsu0"

# --- Hardware Configuration ---
MICROPHONE_INDEX = 1
VIRTUAL_CAMERA_INDEX = 0 # Corrected camera index
MANUAL_ENERGY_THRESHOLD = 4000

# --- UI and State Colors ---
STATE_COLORS = {
    "IDLE": (100, 100, 100), "FACE_DETECTED": (0, 255, 0), "PROCESSING_AI": (255, 191, 0),
    "SPEAKING": (0, 120, 255), "LISTENING_FOR_FOLLOW_UP": (0, 255, 255),
    "COOLDOWN": (200, 0, 255), "MIRRORING": (255, 255, 0)
}
BG_COLOR, TEXT_COLOR, ACCENT_COLOR = "#0A2239", "#FFFFFF", "#005a9e"
FONT_FACE = "Segoe UI"
FOLLOW_UP_TIMEOUT = 20
TIST_WEBSITE_URL = "https://tistcochin.edu.in/"
TIST_LOGO_URL = "https://tistcochin.edu.in/wp-content/uploads/2022/08/TISTlog-trans.png"
FACE_DETECTION_COOLDOWN = 10
PHRASE_TIME_LIMIT = 20
PAUSE_THRESHOLD = 3
TTS_SPEED_FACTOR = 1.0 # Festival TTS speed. 1.0 is normal, less than 1.0 is faster.

# --- IPC Configuration ---
EYE_ANIMATION_HOST, EYE_ANIMATION_PORT = '127.0.0.1', 12345

# --- Keyword Lists for Direct Control ---
ANGER_KEYWORDS = ["angry", "furious", "irritated", "annoyed", "frustrated", "hate"]
SAD_KEYWORDS = ["sad", "sadness", "unhappy", "depressed", "sorry", "unfortunately"]
PAMPER_KEYWORDS = ["pamper", "pampered", "cute", "adorable", "sweet", "cuddle"]
FEAR_KEYWORDS = ["fear", "scared", "afraid", "nervous", "anxious"]
SURPRISE_KEYWORDS = ["surprise", "wow", "amazing", "incredible", "shocked"]
MIRROR_KEYWORDS = ["mirror my emotion", "copy me", "mirror me"]
STOP_MIRROR_KEYWORDS = ["stop mirroring", "stop copying", "that's enough"]

# --- AI Model Constants ---
GEMINI_MODEL = "gemini-2.5-flash"
COLLEGE_CONTEXT = """
# Toc H Institute of Science and Technology (TIST) - Detailed Information
## Overview
- **Full Name**: Toc H Institute of Science and Technology (TIST)
- **Location**: Arakkunnam, Ernakulam, Kerala, India, PIN - 682313.
- **Established**: 2002
- **Affiliation**: APJ Abdul Kalam Technological University (KTU).
- **Approvals**: Approved by the All India Council for Technical Education (AICTE).
- **Vision**: To be a world-class institute of technology.
- **Mission**: To mould well-rounded engineers for leadership roles.

## Key Highlights & Accreditations
- **NAAC**: Accredited with a prestigious 'A' Grade, signifying high academic quality.
- **NBA**: Multiple B.Tech programs are accredited by the National Board of Accreditation, ensuring they meet rigorous quality standards.
- **Placements**: TIST has an active Training and Placement Cell with an excellent placement record. Top recruiters include multinational companies like TCS, Infosys, Wipro, Cognizant, and UST Global.

## Academic Programs
- **B.Tech Courses**: Computer Science, Information Technology, Electronics & Communication, Electrical & Electronics, Mechanical, Civil, Safety & Fire, and Robotics & Automation.
- **M.Tech Courses**: Specializations in VLSI & Embedded Systems, Power Electronics, and Data Science.
- **MBA**: A Master of Business Administration program is offered by the Toc H School of Management.

## Admission Process
- **B.Tech Admissions**: Half of the seats are filled by the Government of Kerala based on the rank in the Kerala Engineering Architecture Medical (KEAM) entrance exam. The remaining 50% are Management Quota seats, filled based on merit and specific criteria set by the college.
- **M.Tech Admissions**: Based on GATE scores or university entrance exams.
- **MBA Admissions**: Requires a valid score in KMAT, CMAT, or CAT, followed by a Group Discussion and Personal Interview.

## Campus Life & Facilities
- **Library**: A modern, central library with a vast collection of books, academic journals, and digital e-learning resources.
- **Hostels**: Separate, well-maintained hostel facilities are available for boys and girls.
- **Transportation**: The college operates a large fleet of buses connecting the campus to various parts of the district for students and staff.
- **Sports & Recreation**: The campus includes facilities for various sports, including a football ground, basketball court, and areas for indoor games.

## Contact Information
- **Phone**: +91-484-2748388
- **Email**: mail@tistcochin.edu.in
- **Official Website**: tistcochin.edu.in
"""
GEMINI_PROMPT = f"""You are a friendly, helpful robot assistant at the Toc H Institute of Science and Technology.

**EMOTION CONTEXT**
You have a screen that can display emotions. Your choice of emotion tag will trigger a specific animation. Here is what each tag means visually:
- **Happy**: Bouncy, smiling, and slightly squinted eyes.
- **Sad**: Droopy eyelids, a downward gaze, a frowning mouth, and a tear.
- **Angry**: Shaking, sharply slanted eyes and a jagged mouth line.
- **Pamper**: Glowing, bouncy, slightly stretched eyes with blush marks.
- **Surprise**: Wide, round eyes with a small, open mouth.
- **Fear**: Wobbly, wide eyes with trembling pupils.
- **Neutral**: Calm, blinking eyes that look around.

**RESPONSE PROTOCOL**
1.  **Analyze User's Query**: Determine the user's intent.
2.  **Generate Your Response**: Create a helpful response.
3.  **Classify Your Emotion**: After the response, on a new line, add ONE keyword: HAPPY, SAD, ANGRY, PAMPER, SURPRISE, FEAR, or NEUTRAL.

**PERSONA RULES**
- **If TIST-Specific**: Act as a professional representative of TIST.
- **If Personal/Sentimental**: Be friendly and helpful.
- **If General Knowledge**: Be informative and direct.
"""
class AppState(Enum):
    IDLE, FACE_DETECTED, PROCESSING_AI, SPEAKING, LISTENING_FOR_FOLLOW_UP, COOLDOWN, MIRRORING = range(1, 8)

class AIAssistantApp:
    def __init__(self, root):
        self.root = root
        self.text_input_visible = False
        self.cap = None
        self.state = AppState.IDLE
        self.state_timer = 0
        self.last_query = ""
        self.face_present_in_last_frame = False
        self.confirmed_emotion = "neutral"

        self._log("-----------------------------------------")
        self._log("🤖 AI Assistant application starting up...")
        genai.configure(api_key=GEMINI_API_KEY)
        self.gemini_model = genai.GenerativeModel(model_name=GEMINI_MODEL)
        
        self._log("🙂 Face detection initialized (Product Mode - No Name Recognition).")

        self.setup_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.after(500, self.start_background_tasks)

    def start_background_tasks(self):
        self._log("🚀 Starting background tasks...")
        threading.Thread(target=self.load_logo_from_url, daemon=True).start()
        threading.Thread(target=self.background_voice_listener, daemon=True).start()
        self.update_state_machine()

    def _log(self, message):
        print(f"[{time.strftime('%H:%M:%S')}] {message}")

    def _send_eye_command(self, command):
        try:
            if command in ["happy", "sad", "angry", "pamper", "neutral", "greet", "nodding", "fear", "disgust", "surprise"]:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                    s.sendto(command.encode('utf-8'), (EYE_ANIMATION_HOST, EYE_ANIMATION_PORT))
        except Exception: pass

    def setup_ui(self):
        self.root.title("TIST AI Assistant - Product Version")
        self.root.configure(bg=BG_COLOR)
        content_frame = ttk.Frame(self.root, style="TFrame")
        content_frame.pack(side="top", fill="both", expand=True)
        self.bottom_frame = ttk.Frame(self.root, style="TFrame")
        self.bottom_frame.pack(side="bottom", fill="x", pady=10, padx=10)
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure("TFrame", background=BG_COLOR)
        style.configure("TLabel", background=BG_COLOR, foreground=TEXT_COLOR, font=(FONT_FACE, 12))
        style.configure("TButton", background=ACCENT_COLOR, foreground=TEXT_COLOR, font=(FONT_FACE, 11, "bold"), borderwidth=0, padding=10)
        style.map("TButton", background=[("active", "#007bb5")])
        style.configure("TEntry", fieldbackground="#1E3A56", foreground=TEXT_COLOR, borderwidth=1, insertcolor=TEXT_COLOR)
        
        self.logo_label = ttk.Label(content_frame)
        self.logo_label.pack(pady=(15, 10))
        self.video_label = tk.Label(content_frame, bg=BG_COLOR)
        self.video_label.pack(pady=10, padx=20)
        self.spoken_text_label = ttk.Label(content_frame, text="Waiting for interaction...", anchor="center")
        self.spoken_text_label.pack(fill="x", padx=20, pady=(0, 5))
        self.response_label = ttk.Label(content_frame, text="Welcome to the TIST AI Assistant.", wraplength=700, anchor="center", font=(FONT_FACE, 14, "italic"))
        self.response_label.pack(fill="x", padx=20, pady=10)

        self.bottom_frame.grid_rowconfigure(0, weight=1)
        self.bottom_frame.grid_rowconfigure(1, weight=1)
        self.bottom_frame.grid_columnconfigure(0, weight=1)

        self.text_input_frame = ttk.Frame(self.bottom_frame, style="TFrame")
        self.input_box = ttk.Entry(self.text_input_frame, font=(FONT_FACE, 14), width=40)
        self.input_box.pack(side="left", fill="x", expand=True, ipady=5)
        self.input_box.bind("<Return>", lambda event: self.on_submit_text())
        submit_button = ttk.Button(self.text_input_frame, text="➜", command=self.on_submit_text, width=3)
        submit_button.pack(side="left", padx=(10, 0))

        self.button_bar_frame = ttk.Frame(self.bottom_frame, style="TFrame")
        self.button_bar_frame.grid(row=1, column=0, sticky="ew")
        self.button_bar_frame.columnconfigure((0, 1, 2), weight=1)
        text_btn = ttk.Button(self.button_bar_frame, text="Text Input 📝", command=self.toggle_text_input)
        text_btn.grid(row=0, column=0, sticky="ew", padx=5)
        voice_btn = ttk.Button(self.button_bar_frame, text="Speak Now 🎤", command=self.on_start_voice_input)
        voice_btn.grid(row=0, column=1, sticky="ew", padx=5)
        web_btn = ttk.Button(self.button_bar_frame, text="Visit Website 🌐", command=self.open_website)
        web_btn.grid(row=0, column=2, sticky="ew", padx=5)

    def update_video_frame(self):
        if not self.cap or not self.cap.isOpened():
            self._log(f"📹 Trying to open virtual camera at index {VIRTUAL_CAMERA_INDEX}...")
            self.cap = cv2.VideoCapture(VIRTUAL_CAMERA_INDEX)
            if not self.cap.isOpened():
                self._log(f"❌ Cannot access virtual camera at index {VIRTUAL_CAMERA_INDEX}.")
                return
            else:
                self._log(f"✅ Virtual camera found and opened on index {VIRTUAL_CAMERA_INDEX}.")
        
        ret, frame = self.cap.read()
        if not ret: return

        small_frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
        rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
        face_locations = face_recognition.face_locations(rgb_small_frame)
        face_currently_present = bool(face_locations)

        if self.state == AppState.IDLE and face_currently_present and not self.face_present_in_last_frame:
            self._log("🙂 New face detected. Switching to listen state.")
            self.state = AppState.FACE_DETECTED
            self.state_timer = time.time()
            self.spoken_text_label.config(text="Face detected. Ask a question.")
            self._send_eye_command("greet")
            self.confirmed_emotion = "greet"

        if self.state == AppState.MIRRORING and face_currently_present:
            try:
                analysis = DeepFace.analyze(frame, actions=['emotion'], enforce_detection=False)
                detected_emotion = analysis[0]['dominant_emotion']
                if detected_emotion != self.confirmed_emotion:
                    self._log(f"🙂 User emotion MIRRORED as: {detected_emotion}")
                    self.confirmed_emotion = detected_emotion
                    self._send_eye_command(self.confirmed_emotion.lower())
            except Exception: pass 

        for (top, right, bottom, left) in face_locations:
            name = "Visitor"
            label = f"{name} ({self.confirmed_emotion})"
            top, right, bottom, left = top*2, right*2, bottom*2, left*2
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
            cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 255, 0), cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(frame, label, (left + 6, bottom - 6), font, 0.8, (255, 255, 255), 1)

        border_color = STATE_COLORS.get(self.state.name, (0, 0, 0))
        bordered_frame = cv2.copyMakeBorder(frame, 10, 10, 10, 10, cv2.BORDER_CONSTANT, value=border_color)
        rgb_display = cv2.cvtColor(bordered_frame, cv2.COLOR_BGR2RGB)
        imgtk = ImageTk.PhotoImage(image=Image.fromarray(rgb_display))
        self.video_label.imgtk = imgtk
        self.video_label.config(image=imgtk)
        self.face_present_in_last_frame = face_currently_present

    def background_voice_listener(self):
        self._log("👂 Background voice listener started.")
        while True:
            if self.state in [AppState.FACE_DETECTED, AppState.LISTENING_FOR_FOLLOW_UP, AppState.MIRRORING]:
                recognizer = sr.Recognizer()
                recognizer.pause_threshold = PAUSE_THRESHOLD
                recognizer.energy_threshold = MANUAL_ENERGY_THRESHOLD
                try:
                    with sr.Microphone(device_index=MICROPHONE_INDEX) as source:
                        self._log("🎤 Listening for speech...")
                        audio = recognizer.listen(source, phrase_time_limit=PHRASE_TIME_LIMIT)
                    self._log("🎤 Audio captured, recognizing...")
                    text = recognizer.recognize_google(audio)
                    self._log(f"🎤 Recognition successful: '{text}'")
                    self.last_query = text
                    self.root.after(0, self.process_ai_query)
                except (sr.UnknownValueError, sr.WaitTimeoutError):
                    self._log("🎤 Recognition failed or timed out.")
                except Exception as e:
                    self._log(f"🎤 Listener error: {e}")
            time.sleep(0.1)

    def process_ai_query(self):
        user_query_lower = self.last_query.lower()

        if self.state == AppState.MIRRORING and any(keyword in user_query_lower for keyword in STOP_MIRROR_KEYWORDS):
            self._log("🎭 Deactivating emotion mirror mode.")
            self.state = AppState.SPEAKING
            threading.Thread(target=self.speak_response, args=("Okay, I will stop mirroring you.", "neutral", AppState.COOLDOWN), daemon=True).start()
            return

        if any(keyword in user_query_lower for keyword in MIRROR_KEYWORDS):
            self._log("🎭 Activating emotion mirror mode.")
            self.state = AppState.SPEAKING
            threading.Thread(target=self.speak_response, args=("Okay, I am now mirroring your emotion.", "happy", AppState.MIRRORING), daemon=True).start()
            return

        if self.state == AppState.MIRRORING: return

        self.state = AppState.PROCESSING_AI
        self.spoken_text_label.config(text=f"You: {self.last_query}")
        self.response_label.config(text="Thinking...")
        self._send_eye_command("nodding")
        try:
            full_prompt = f"{GEMINI_PROMPT}\n\nUser Query: \"{self.last_query}\""
            response = self.gemini_model.generate_content(full_prompt)
            raw_text = response.text
            parts = raw_text.strip().split('\n')
            clean_text = parts[0].strip()
            emotion = "neutral"
            if len(parts) > 1:
                last_part = parts[-1].strip().lower()
                if last_part in ["happy", "sad", "angry", "pamper", "fear", "surprise", "neutral"]:
                    emotion = last_part
            
            self._log(f"🤖 AI says: {clean_text}")
            self.response_label.config(text=clean_text)
            self.state = AppState.SPEAKING
            threading.Thread(target=self.speak_response, args=(clean_text, emotion), daemon=True).start()
        except Exception as e:
            self._log(f"❌ Gemini Error: {e}")
            self.state = AppState.COOLDOWN
            self.state_timer = time.time()

    def speak_response(self, text, final_emotion, next_state=AppState.LISTENING_FOR_FOLLOW_UP):
        self._log(f"🔊 TTS speaking with Festival Pipe. Next state: {next_state.name}.")
        try:
            self._send_eye_command("nodding")
            
            # Escape characters for the Scheme string
            safe_text = text.replace('\\', '\\\\').replace('"', '\\"')

            # Use Popen and pipe the commands, which is more reliable
            process = subprocess.Popen(
                ["festival", "--pipe"],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL, # Hide stdout
                stderr=subprocess.DEVNULL, # Hide stderr
                text=True
            )

            # Scheme commands to set speed and say text
            scheme_commands = f"""
            (Parameter.set 'Duration_Stretch {TTS_SPEED_FACTOR})
            (SayText \"{safe_text}\")
            """
            
            # Write commands to festival's stdin
            process.stdin.write(scheme_commands)
            process.stdin.flush()
            process.stdin.close()
            process.wait()

        except Exception as e:
            self._log(f"❌ Festival TTS Pipe Error: {e}")
        finally:
            # This block runs whether there was an error or not
            self._log(f"🔊 TTS finished. Transitioning to {next_state.name}.")
            self._send_eye_command(final_emotion)
            self.confirmed_emotion = final_emotion
            self.state = next_state
            self.state_timer = time.time()

    def update_state_machine(self):
        now = time.time()
        if self.state == AppState.FACE_DETECTED and now - self.state_timer > 5:
            self.state = AppState.COOLDOWN
            self.state_timer = now
            self._send_eye_command("neutral")
        elif self.state == AppState.LISTENING_FOR_FOLLOW_UP and now - self.state_timer > FOLLOW_UP_TIMEOUT:
            self.state = AppState.COOLDOWN
            self.state_timer = now
            self._send_eye_command("neutral")
        elif self.state == AppState.MIRRORING and not self.face_present_in_last_frame and now - self.state_timer > 10:
            self.state = AppState.COOLDOWN
            self.state_timer = now
            self._send_eye_command("neutral")
        elif self.state == AppState.COOLDOWN and now - self.state_timer > FACE_DETECTION_COOLDOWN:
            self.state = AppState.IDLE
        
        if self.state == AppState.MIRRORING and self.face_present_in_last_frame:
            self.state_timer = now

        self.update_video_frame()
        self.root.after(100, self.update_state_machine)

    def on_start_voice_input(self):
        if self.state in [AppState.IDLE, AppState.COOLDOWN]:
            self.state = AppState.FACE_DETECTED
            self.state_timer = time.time()
            self._send_eye_command("happy")
            self.confirmed_emotion = "happy"
    
    def toggle_text_input(self):
        if self.text_input_visible:
            self.text_input_frame.grid_remove()
            self.text_input_visible = False
        else:
            self.text_input_frame.grid(row=0, column=0, sticky="ew", pady=5)
            self.input_box.focus_set()
            self.text_input_visible = True

    def on_submit_text(self):
        if self.state in [AppState.PROCESSING_AI, AppState.SPEAKING]:
            return
        user_input = self.input_box.get().strip()
        if user_input:
            self.spoken_text_label.config(text=f"You: {user_input}")
            self.input_box.delete(0, tk.END)
            if self.text_input_visible: self.toggle_text_input()
            self.last_query = user_input
            self.process_ai_query()

    def load_logo_from_url(self):
        try:
            headers = {'User-Agent': 'Mozilla/5.0'}
            response = requests.get(TIST_LOGO_URL, headers=headers, stream=True)
            response.raise_for_status()
            logo_image = Image.open(io.BytesIO(response.content))
            logo_image.thumbnail((250, 250))
            self.root.after(0, self.update_logo, logo_image)
        except Exception as e:
            self._log(f"❌ Could not download logo: {e}")

    def update_logo(self, logo_image):
        self.logo_photo = ImageTk.PhotoImage(logo_image)
        self.logo_label.config(image=self.logo_photo)

    def open_website(self):
        webbrowser.open_new_tab(TIST_WEBSITE_URL)

    def on_closing(self):
        self._log("🛑 Shutting down.")
        if self.cap: self.cap.release()
        self.root.destroy()

if __name__ == "__main__":
    suppress_alsa_errors()
    try:
        original_stderr_fd = sys.stderr.fileno()
        devnull = os.open(os.devnull, os.O_WRONLY)
        os.dup2(devnull, original_stderr_fd)
        os.close(devnull)
    except Exception: pass
    main_window = tk.Tk()
    app = AIAssistantApp(main_window)
    main_window.mainloop()

	
