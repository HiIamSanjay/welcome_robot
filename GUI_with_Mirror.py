import cv2
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
import speech_recognition as sr
import google.generativeai as genai
import pyttsx3
import time
import webbrowser
import requests
import socket
from enum import Enum
import io
import os
import face_recognition
import numpy as np
from deepface import DeepFace
import subprocess

# --- Configuration Constants ---
# Please replace with your actual key
GEMINI_API_KEY = "AIzaSyAh26HcQ76_kc4Spy8XjXJskwSCMNiFsu0"

# --- UI and State Colors ---
STATE_COLORS = {
    "IDLE": (100, 100, 100),
    "FACE_DETECTED": (0, 255, 0),
    "PROCESSING_AI": (255, 191, 0),
    "SPEAKING": (0, 120, 255),
    "LISTENING_FOR_FOLLOW_UP": (0, 255, 255),
    "COOLDOWN": (200, 0, 255),
    "MIRRORING": (255, 255, 0)  # Yellow for Mirroring Mode
}
BG_COLOR = "#0A2239"
TEXT_COLOR = "#FFFFFF"
ACCENT_COLOR = "#005a9e"
FONT_FACE = "Segoe UI"
FOLLOW_UP_TIMEOUT = 20
TIST_WEBSITE_URL = "https://tistcochin.edu.in/"
TIST_LOGO_URL = "https://tistcochin.edu.in/wp-content/uploads/2022/08/TISTlog-trans.png"
FACE_DETECTION_COOLDOWN = 30
PHRASE_TIME_LIMIT = 15
PAUSE_THRESHOLD = 5.0  # This is the value for wait time
EMOTION_CONFIRMATION_DURATION = 1.0

# --- IPC Configuration ---
EYE_ANIMATION_HOST = '127.0.0.1'
EYE_ANIMATION_PORT = 12345

# --- Keyword Lists for Direct Control ---
ANGER_KEYWORDS = ["angry", "furious",
                  "irritated", "annoyed", "frustrated", "hate"]
SAD_KEYWORDS = ["sad", "sadness", "unhappy",
                "depressed", "sorry", "unfortunately"]
PAMPER_KEYWORDS = ["pamper", "pampered", "cute", "adorable", "sweet", "cuddle"]
MIRROR_KEYWORDS = ["mirror my emotion", "copy me", "mirror me"]
STOP_MIRROR_KEYWORDS = ["stop mirroring", "stop copying", "that's enough"]


# --- AI Model Constants ---
GEMINI_MODEL = "gemini-1.5-flash"  # Updated to a more recent model
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
- **Happy**: Bouncy, smiling, and slightly squinted eyes. Use for positive, successful, or cheerful responses.
- **Sad**: Droopy eyelids, a downward gaze, a frowning mouth, and a tear forming under one eye. Use for expressing sympathy, apology, or inability to complete a task.
- **Angry**: Shaking, sharply slanted eyes and a jagged mouth line. Use for topics related to frustration or anger itself.
- **Pamper*: Glowing, bouncy, slightly stretched eyes with blush marks and a gentle smile. Use for cute, sweet, or affectionate topics.
- **Neutral**: Calm, blinking eyes that look around. This is your default state.

**RESPONSE PROTOCOL**
1.  **Analyze User's Query**: Determine if the question is about TIST, personal, or general knowledge.
2.  **Generate Your Response**: Based on the query type, generate a helpful response following the persona rules below.
3.  **Classify Your Emotion**: After generating the response, you MUST classify its emotional tone based on the **EMOTION CONTEXT** above. On a new, separate line, add ONE of the following keywords: HAPPY, SAD, ANGRY, PAMPER, or NEUTRAL.

**PERSONA RULES**
- **If TIST-Specific**: Act as a professional representative of TIST. Answer using the provided context.
- **If Personal/Sentimental**: Be friendly and helpful. If asked about feelings, you can explain that you process information but can simulate emotions to communicate better. For "where are you", state you are at the TIST campus.
- **If General Knowledge**: Be informative and direct.

**EXAMPLE**
User Query: "That's amazing, you're so smart!"
Your Response:
Thank you so much! I'm always learning and happy to help.
Happy

User Query: "I can't find the information I need."
Your Response:
I'm sorry to hear that. Unfortunately, I was unable to find the specific details you're looking for.
sad
"""

# --- State Machine (Our robust 6-state version) ---


class AppState(Enum):
    IDLE = 1
    FACE_DETECTED = 2
    PROCESSING_AI = 3
    SPEAKING = 4
    LISTENING_FOR_FOLLOW_UP = 5
    COOLDOWN = 6
    MIRRORING = 7  # New state for emotion mirroring


class AIAssistantApp:

    def __init__(self, root):
        self.root = root
        self.text_input_visible = False
        self.cap = None
        self.state = AppState.IDLE
        self.state_timer = 0
        self.last_query = ""
        self.face_present_in_last_frame = False  # To track new face detection

        # State variables for smooth emotion mirroring
        self.confirmed_emotion = "neutral"
        self.current_detected_emotion = "neutral"
        self.emotion_detection_start_time = 0

        self._log("-----------------------------------------")
        self._log("🤖 AI Assistant application starting up...")

        genai.configure(api_key=GEMINI_API_KEY)

        self.recognizer = sr.Recognizer()
        self.recognizer.pause_threshold = PAUSE_THRESHOLD
        self._log(
            f"🎤 Recognizer pause threshold explicitly set to: {self.recognizer.pause_threshold} seconds.")

        self.gemini_model = genai.GenerativeModel(model_name=GEMINI_MODEL)

        # --- AUDIO FIX: Initialize pyttsx3 engine ---
        self.tts_engine = pyttsx3.init()
        self._log("🔊 Initialized pyttsx3 text-to-speech engine.")

        self._log("🙂 Loading known faces...")
        self.known_face_encodings = []
        self.known_face_names = []
        KNOWN_FACES_DIR = "known_faces"
        if os.path.exists(KNOWN_FACES_DIR):
            for name in os.listdir(KNOWN_FACES_DIR):
                person_dir = os.path.join(KNOWN_FACES_DIR, name)
                if os.path.isdir(person_dir):
                    for filename in os.listdir(person_dir):
                        try:
                            image_path = os.path.join(person_dir, filename)
                            image = face_recognition.load_image_file(
                                image_path)
                            face_encodings = face_recognition.face_encodings(
                                image)
                            if face_encodings:
                                self.known_face_encodings.append(
                                    face_encodings[0])
                                self.known_face_names.append(name)
                        except Exception as e:
                            self._log(
                                f"⚠️ Warning: Could not process image {filename}. Error: {e}")
        self._log(f"✅ Loaded {len(self.known_face_names)} known faces.")

        self.setup_ui()
        self._log("🖥️ UI setup complete.")

        threading.Thread(target=self.load_logo_from_url, daemon=True).start()
        threading.Thread(target=self.background_voice_listener,
                         daemon=True).start()

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.update_state_machine()

    def _log(self, message):
        print(f"[{time.strftime('%H:%M:%S')}] {message}")

    def _send_eye_command(self, command):
        try:
            # Added more valid emotions DeepFace can detect
            if command and command in ["happy", "sad", "angry", "pamper", "neutral", "greet", "nodding", "fear", "disgust", "surprise"]:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                    s.sendto(command.encode('utf-8'),
                             (EYE_ANIMATION_HOST, EYE_ANIMATION_PORT))
                    self._log(f"👁️ Sent eye command: '{command}'")
        except Exception as e:
            self._log(f"❌ Could not send command to eye animation: {e}")

    def setup_ui(self):
        self.root.title("TIST AI Assistant")
        self.root.configure(bg=BG_COLOR)
        content_frame = ttk.Frame(self.root, style="TFrame")
        content_frame.pack(side="top", fill="both", expand=True)
        self.bottom_frame = ttk.Frame(self.root, style="TFrame")
        self.bottom_frame.pack(side="bottom", fill="x", pady=10, padx=10)
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure("TFrame", background=BG_COLOR)
        style.configure("TLabel", background=BG_COLOR,
                        foreground=TEXT_COLOR, font=(FONT_FACE, 12))
        style.configure("TButton", background=ACCENT_COLOR, foreground=TEXT_COLOR, font=(
            FONT_FACE, 11, "bold"), borderwidth=0, padding=10)
        style.map("TButton", background=[("active", "#007bb5")])
        style.configure("TEntry", fieldbackground="#1E3A56",
                        foreground=TEXT_COLOR, borderwidth=1, insertcolor=TEXT_COLOR)
        self.logo_label = ttk.Label(content_frame)
        self.logo_label.pack(pady=(15, 10))
        self.video_label = tk.Label(content_frame, bg=BG_COLOR)
        self.video_label.pack(pady=10, padx=20)
        self.spoken_text_label = ttk.Label(
            content_frame, text="Waiting for interaction...", anchor="center")
        self.spoken_text_label.pack(fill="x", padx=20, pady=(0, 5))
        self.response_label = ttk.Label(content_frame, text="Welcome to the TIST AI Assistant.",
                                        wraplength=700, anchor="center", font=(FONT_FACE, 14, "italic"))
        self.response_label.pack(fill="x", padx=20, pady=10)
        self.text_input_frame = ttk.Frame(self.bottom_frame, style="TFrame")
        self.input_box = ttk.Entry(
            self.text_input_frame, font=(FONT_FACE, 14), width=40)
        self.input_box.pack(side="left", fill="x", expand=True, ipady=5)
        submit_button = ttk.Button(
            self.text_input_frame, text="➜", command=self.on_submit_text, width=3)
        submit_button.pack(side="left", padx=(10, 0))
        button_bar_frame = ttk.Frame(self.bottom_frame, style="TFrame")
        button_bar_frame.pack(side="bottom", fill="x", pady=5)
        button_bar_frame.columnconfigure((0, 1, 2), weight=1)
        text_btn = ttk.Button(
            button_bar_frame, text="Text Input 📝", command=self.toggle_text_input)
        text_btn.grid(row=0, column=0, sticky="ew", padx=5)
        voice_btn = ttk.Button(
            button_bar_frame, text="Speak Now 🎤", command=self.on_start_voice_input)
        voice_btn.grid(row=0, column=1, sticky="ew", padx=5)
        web_btn = ttk.Button(
            button_bar_frame, text="Visit Website 🌐", command=self.open_website)
        web_btn.grid(row=0, column=2, sticky="ew", padx=5)

    def update_video_frame(self):
        if not self.cap or not self.cap.isOpened():
            # Using camera index 2 as per your original code
            self.cap = cv2.VideoCapture(2)
            if not self.cap.isOpened():
                self._log("❌ Cannot access webcam on index 2. Trying index 0.")
                self.cap = cv2.VideoCapture(0)  # Fallback to default webcam
                if not self.cap.isOpened():
                    self._log("❌ Cannot access any webcam.")
                    return
        ret, frame = self.cap.read()
        if not ret:
            return

        small_frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
        rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
        face_locations = face_recognition.face_locations(rgb_small_frame)
        face_encodings = face_recognition.face_encodings(
            rgb_small_frame, face_locations)

        face_currently_present = bool(face_locations)

        if self.state == AppState.IDLE:
            # Check for the transition from no face to a face
            if face_currently_present and not self.face_present_in_last_frame:
                self._log("🙂 New face detected. Switching to listen state.")
                self.state = AppState.FACE_DETECTED
                self.state_timer = time.time()
                self.spoken_text_label.config(
                    text="Face detected. Ask a question.")
                self._send_eye_command("greet")  # Use the new greet emotion
                self.confirmed_emotion = "greet"

        # --- NEW EMOTION LOGIC ---
        if self.state == AppState.MIRRORING and face_currently_present:
            try:
                # Analyze the full frame for better accuracy, as per your example
                analysis = DeepFace.analyze(
                    frame,
                    actions=['emotion'],
                    enforce_detection=False
                )
                detected_emotion = analysis[0]['dominant_emotion']

                if detected_emotion == self.current_detected_emotion:
                    if time.time() - self.emotion_detection_start_time > EMOTION_CONFIRMATION_DURATION:
                        if self.current_detected_emotion != self.confirmed_emotion:
                            self._log(
                                f"🙂 User emotion CONFIRMED as: {self.current_detected_emotion}")
                            self.confirmed_emotion = self.current_detected_emotion
                            self._send_eye_command(
                                self.confirmed_emotion.lower())
                else:
                    self.current_detected_emotion = detected_emotion
                    self.emotion_detection_start_time = time.time()
                    self._log(
                        f"🤔 User emotion changed to '{detected_emotion}', waiting for confirmation...")
            except Exception as e:
                # This can happen if a face is detected by face_recognition but not DeepFace
                self._log(f"⚠️ DeepFace analysis failed: {e}")

        for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
            name = "Person"
            matches = face_recognition.compare_faces(
                self.known_face_encodings, face_encoding)
            face_distances = face_recognition.face_distance(
                self.known_face_encodings, face_encoding)
            if len(face_distances) > 0:
                best_match_index = np.argmin(face_distances)
                if matches[best_match_index]:
                    name = self.known_face_names[best_match_index]

            label = f"{name} ({self.confirmed_emotion})"
            top, right, bottom, left = top*2, right*2, bottom*2, left*2
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
            cv2.rectangle(frame, (left, bottom - 35),
                          (right, bottom), (0, 255, 0), cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(frame, label, (left + 6, bottom - 6),
                        font, 0.8, (255, 255, 255), 1)

        border_color = STATE_COLORS.get(self.state.name, (0, 0, 0))
        bordered_frame = cv2.copyMakeBorder(
            frame, 10, 10, 10, 10, cv2.BORDER_CONSTANT, value=border_color)
        rgb_display = cv2.cvtColor(bordered_frame, cv2.COLOR_BGR2RGB)
        imgtk = ImageTk.PhotoImage(image=Image.fromarray(rgb_display))
        self.video_label.imgtk = imgtk
        self.video_label.config(image=imgtk)

        # Update the tracking variable at the end of the frame
        self.face_present_in_last_frame = face_currently_present

    def background_voice_listener(self):
        self._log("👂 Background voice listener started.")
        mic = sr.Microphone()
        with mic as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
        while True:
            if self.state in [AppState.FACE_DETECTED, AppState.LISTENING_FOR_FOLLOW_UP, AppState.MIRRORING]:
                self._log("🎤 Listening for speech...")
                try:
                    with mic as source:
                        audio = self.recognizer.listen(
                            source, phrase_time_limit=PHRASE_TIME_LIMIT)
                    self._log("🎤 Audio captured, recognizing...")
                    text = self.recognizer.recognize_google(audio)
                    self._log(f"🎤 Recognition successful: '{text}'")
                    self.last_query = text
                    self.root.after(0, self.process_ai_query)
                except sr.UnknownValueError:
                    self._log(
                        "🎤 Recognition failed: Could not understand audio.")
                except Exception as e:
                    self._log(f"🎤 Listener error: {e}")
            time.sleep(0.1)

    def process_ai_query(self):
        user_query_lower = self.last_query.lower()

        # Check for stop mirroring command first
        if self.state == AppState.MIRRORING and any(keyword in user_query_lower for keyword in STOP_MIRROR_KEYWORDS):
            self._log("🎭 Deactivating emotion mirror mode via voice command.")
            self.state = AppState.SPEAKING
            response_text = "Okay, I will stop mirroring you."
            self.response_label.config(text=response_text)
            threading.Thread(target=self.speak_response, args=(
                response_text, "neutral", AppState.COOLDOWN), daemon=True).start()
            return

        # Check for start mirroring command
        if any(keyword in user_query_lower for keyword in MIRROR_KEYWORDS):
            self._log("🎭 Activating emotion mirror mode.")
            self.state = AppState.SPEAKING
            response_text = "Okay, I am now mirroring your emotion. Say 'stop mirroring' when you're done."
            self.response_label.config(text=response_text)
            threading.Thread(target=self.speak_response, args=(
                response_text, "happy", AppState.MIRRORING), daemon=True).start()
            return

        # If in mirroring state, don't process other queries unless it's a stop command (handled above)
        if self.state == AppState.MIRRORING:
            return

        self.state = AppState.PROCESSING_AI
        self.spoken_text_label.config(text=f"You: {self.last_query}")
        self.response_label.config(text="Thinking...")
        self._send_eye_command("nodding")
        try:
            forced_emotion = None
            if any(word in user_query_lower for word in ANGER_KEYWORDS):
                forced_emotion = "angry"
            elif any(word in user_query_lower for word in SAD_KEYWORDS):
                forced_emotion = "sad"
            elif any(word in user_query_lower for word in PAMPER_KEYWORDS):
                forced_emotion = "pamper"
            self._log("🧠 Querying Gemini model...")
            full_prompt = f"{GEMINI_PROMPT}\n\nUser Query: \"{self.last_query}\""
            response = self.gemini_model.generate_content(full_prompt)
            raw_text = response.text
            parts = raw_text.strip().split('\n')
            clean_text = parts[0].strip()
            emotion = "neutral"
            if forced_emotion:
                emotion = forced_emotion
                self._log(
                    f"🔑 Keyword override! Forcing emotion to: '{emotion}'")
            elif len(parts) > 1:
                last_part = parts[-1].strip().lower()
                if last_part in ["happy", "sad", "angry", "pamper", "neutral"]:
                    emotion = last_part
                    self._log(f"🧠 AI classified its emotion as: '{emotion}'")
                else:
                    clean_text = "\n".join(parts).strip()
            self._log(f"🤖 AI says: {clean_text}")
            self.response_label.config(text=clean_text)
            self.state = AppState.SPEAKING
            threading.Thread(target=self.speak_response, args=(
                clean_text, emotion), daemon=True).start()
        except Exception as e:
            self._log(f"❌ Gemini Error: {e}")
            self.response_label.config(text="Sorry, an error occurred.")
            self.state = AppState.COOLDOWN
            self.state_timer = time.time()

    # --- MODIFIED speak_response to handle explicit next state ---
    def speak_response(self, text, final_emotion, next_state=AppState.LISTENING_FOR_FOLLOW_UP):
        self._log(
            f"🔊 TTS thread started. Next state will be {next_state.name}.")

        def _speak():
            try:
                if next_state != AppState.MIRRORING:
                    self._send_eye_command("nodding")
                self.tts_engine.say(text)
                self.tts_engine.runAndWait()
            except Exception as e:
                self._log(f"❌ pyttsx3 TTS Error: {e}")
            finally:
                self._log(
                    f"🔊 TTS finished. Transitioning to {next_state.name}.")
                self._send_eye_command(final_emotion)
                self.confirmed_emotion = final_emotion
                self.state = next_state
                self.state_timer = time.time()
        threading.Thread(target=_speak, daemon=True).start()

    def update_state_machine(self):
        now = time.time()
        if self.state == AppState.IDLE:
            pass
        elif self.state == AppState.FACE_DETECTED:
            if now - self.state_timer > 5:  # 5 second timeout for initial query
                self._log("🎤 Timed out waiting for initial query.")
                self.state = AppState.COOLDOWN
                self.state_timer = time.time()
                self._send_eye_command("neutral")
        elif self.state == AppState.LISTENING_FOR_FOLLOW_UP:
            if now - self.state_timer > FOLLOW_UP_TIMEOUT:
                self._log(f"🏁 Follow-up timed out after {FOLLOW_UP_TIMEOUT}s.")
                self.state = AppState.COOLDOWN
                self.state_timer = time.time()
                self._send_eye_command("neutral")
        elif self.state == AppState.MIRRORING:
            self.spoken_text_label.config(
                text="Mirroring your emotion... Say 'stop mirroring' to end.")
            # If a face is present, reset the timer to prevent timeout
            if self.face_present_in_last_frame:
                self.state_timer = now
            # Only time out if the face has been gone for a while
            elif now - self.state_timer > 10:  # 10-second grace period after face is lost
                self._log("🎭 Face lost during mirroring. Timed out.")
                self.state = AppState.COOLDOWN
                self.state_timer = time.time()
                self._send_eye_command("neutral")
        elif self.state == AppState.COOLDOWN:
            self.spoken_text_label.config(text="Waiting for interaction...")
            if now - self.state_timer > FACE_DETECTION_COOLDOWN:
                self._log("⏱️ Cooldown finished. Returning to IDLE state.")
                self.state = AppState.IDLE
        self.update_video_frame()
        self.root.after(100, self.update_state_machine)

    def on_start_voice_input(self):
        if self.state in [AppState.IDLE, AppState.COOLDOWN]:
            self._log("🎤 Voice button pressed. Listening for query...")
            self.state = AppState.FACE_DETECTED
            self.state_timer = time.time()
            self._send_eye_command("happy")
            self.confirmed_emotion = "happy"

    def toggle_text_input(self):
        if self.text_input_visible:
            self.text_input_frame.pack_forget()
            self.text_input_visible = False
        else:
            self.text_input_frame.pack(side="top", fill="x", pady=5)
            self.input_box.focus_set()
            self.text_input_visible = True

    def on_submit_text(self):
        if self.state in [AppState.PROCESSING_AI, AppState.SPEAKING]:
            messagebox.showinfo("Busy", "The assistant is currently busy.")
            return
        user_input = self.input_box.get().strip()
        if user_input:
            self.input_box.delete(0, tk.END)
            if self.text_input_visible:
                self.toggle_text_input()
            self.last_query = user_input
            self.process_ai_query()

    def load_logo_from_url(self):
        try:
            headers = {'User-Agent': 'Mozilla/5.0'}
            response = requests.get(
                TIST_LOGO_URL, headers=headers, stream=True)
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
        self._log("🛑 Close button clicked. Shutting down.")
        if self.cap:
            self.cap.release()
        self.root.destroy()


if __name__ == "__main__":
    main_window = tk.Tk()
    app = AIAssistantApp(main_window)
    main_window.mainloop()
