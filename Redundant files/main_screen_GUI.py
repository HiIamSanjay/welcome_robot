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

# --- Configuration Constants ---
GEMINI_API_KEY = "AIzaSyDv1KDhOOfv7lH3MT-hJJ8r2SD9oZ4_NXY"

# --- Toc H Themed Constants ---
BG_COLOR = "#0A2239"
TEXT_COLOR = "#FFFFFF"
ACCENT_COLOR = "#005a9e"
FONT_FACE = "Segoe UI"
FOLLOW_UP_TIMEOUT = 20
TIST_WEBSITE_URL = "https://tistcochin.edu.in/"
TIST_LOGO_URL = "https://tistcochin.edu.in/wp-content/uploads/2022/08/TISTlog-trans.png"
FACE_DETECTION_COOLDOWN = 30
# ### MODIFICATION START: Speech Recognition Settings ###
# The maximum duration for a single spoken phrase (in seconds)
PHRASE_TIME_LIMIT = 15 
# The amount of silence to wait for before considering a phrase complete (in seconds)
PAUSE_THRESHOLD = 2.0 
# ### MODIFICATION END ###


# --- IPC Configuration ---
EYE_ANIMATION_HOST = '127.0.0.1'
EYE_ANIMATION_PORT = 12345

# --- Keyword Lists for Direct Control ---
ANGER_KEYWORDS = ["angry", "furious", "irritated", "annoyed", "frustrated", "hate"]
SAD_KEYWORDS = ["sad", "sadness", "unhappy", "depressed", "sorry", "unfortunately"]
PAMPER_KEYWORDS = ["pamper", "pampered", "cute", "adorable", "sweet", "cuddle"]

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
- **B.Tech Admissions**: 50% of seats are filled by the Government of Kerala based on the rank in the Kerala Engineering Architecture Medical (KEAM) entrance exam. The remaining 50% are Management Quota seats, filled based on merit and specific criteria set by the college.
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

# --- State Machine ---
class AppState(Enum):
    IDLE = 1
    FACE_DETECTED = 2
    PROCESSING_AI = 3
    SPEAKING = 4
    LISTENING_FOR_FOLLOW_UP = 5
    COOLDOWN = 6

class AIAssistantApp:
 
    def __init__(self, root):
        self.root = root
        self.text_input_visible = False
        self.cap = None
        self.state = AppState.IDLE
        self.state_timer = 0
        self.last_query = ""
        self._log("-----------------------------------------")
        self._log("🤖 AI Assistant application starting up...")
        if not GEMINI_API_KEY or "YOUR_API_KEY" in GEMINI_API_KEY:
            self._log("❌ FATAL: Gemini API key not found.")
            messagebox.showerror("API Key Error", "Please paste your Gemini API key into the script.")
            root.destroy()
            return
        genai.configure(api_key=GEMINI_API_KEY)

        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        
        self.recognizer = sr.Recognizer()
        self.recognizer.pause_threshold = PAUSE_THRESHOLD
        self._log(f"🎙️ Speech recognizer pause threshold set to {PAUSE_THRESHOLD} seconds.")

        self.gemini_model = genai.GenerativeModel(model_name=GEMINI_MODEL)
        
        # ### MODIFICATION START: Set a Custom Voice ###
        self.tts_engine = pyttsx3.init()
        
        # --- Voice Selection Logic ---
        # ❗ PASTE THE VOICE ID YOU CHOSE FROM THE TEST SCRIPT HERE
        # It will look something like 'english-us' or 'mb-us1'
        # If you can't find a good one, leave it as None to use the default.
        desired_voice_id = "cmu_us_kal_arctic_hts" # Example: "english-mb-mbrola-1" 

        if desired_voice_id:
            try:
                self.tts_engine.setProperty('voice', desired_voice_id)
                self._log(f"🔊 Custom TTS voice set to: {desired_voice_id}")
            except Exception as e:
                self._log(f"⚠️ Could not set custom voice '{desired_voice_id}'. Using default. Error: {e}")
        # ### MODIFICATION END ###

        self.setup_ui()
        self._log("🖥️ UI setup complete.")
        threading.Thread(target=self.load_logo_from_url, daemon=True).start()
        threading.Thread(target=self.background_voice_listener, daemon=True).start()
        self.update_state_machine()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def _log(self, message):
        print(f"[{time.strftime('%H:%M:%S')}] {message}")

    def _send_eye_command(self, command):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.sendto(command.encode('utf-8'), (EYE_ANIMATION_HOST, EYE_ANIMATION_PORT))
                self._log(f"👁️ Sent eye command: '{command}'")
        except Exception as e:
            self._log(f"❌ Could not send command to eye animation: {e}")

    def setup_ui(self):
        # ... (this function remains the same) ...
        self.root.title("TIST AI Assistant")
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
        self.text_input_frame = ttk.Frame(self.bottom_frame, style="TFrame")
        self.input_box = ttk.Entry(self.text_input_frame, font=(FONT_FACE, 14), width=40)
        self.input_box.pack(side="left", fill="x", expand=True, ipady=5)
        submit_button = ttk.Button(self.text_input_frame, text="➜", command=self.on_submit_text, width=3)
        submit_button.pack(side="left", padx=(10, 0))
        button_bar_frame = ttk.Frame(self.bottom_frame, style="TFrame")
        button_bar_frame.pack(side="bottom", fill="x", pady=5)
        button_bar_frame.columnconfigure((0, 1, 2), weight=1)
        text_btn = ttk.Button(button_bar_frame, text="Text Input 📝", command=self.toggle_text_input)
        text_btn.grid(row=0, column=0, sticky="ew", padx=5)
        voice_btn = ttk.Button(button_bar_frame, text="Speak Now 🎤", command=self.on_start_voice_input)
        voice_btn.grid(row=0, column=1, sticky="ew", padx=5)
        web_btn = ttk.Button(button_bar_frame, text="Visit Website 🌐", command=self.open_website)
        web_btn.grid(row=0, column=2, sticky="ew", padx=5)

    def toggle_text_input(self):
        # ... (this function remains the same) ...
        if self.text_input_visible:
            self.text_input_frame.pack_forget()
            self.text_input_visible = False
        else:
            self.text_input_frame.pack(side="top", fill="x", pady=5)
            self.input_box.focus_set()
            self.text_input_visible = True

    def on_submit_text(self):
        # ... (this function remains the same) ...
        if self.state not in [AppState.IDLE, AppState.COOLDOWN]:
            messagebox.showinfo("Busy", "The assistant is currently busy. Please wait.")
            return
        user_input = self.input_box.get().strip()
        if user_input:
            self.input_box.delete(0, tk.END)
            if self.text_input_visible:
                self.toggle_text_input()
            self._log(f"⌨️ Text input received: '{user_input}'")
            self.last_query = user_input
            self.process_ai_query()
        else:
            messagebox.showwarning("Input Missing", "Please enter a question.")
    
    def on_start_voice_input(self):
        # ... (this function remains the same) ...
        if self.state == AppState.IDLE or self.state == AppState.COOLDOWN:
            self._log("🎤 Voice button pressed. Listening for query...")
            self.spoken_text_label.config(text="Listening...")
            self.state = AppState.FACE_DETECTED
            self.state_timer = time.time()
            self._send_eye_command("happy")
        else:
            messagebox.showinfo("Busy", "The assistant is currently busy. Please wait.")

    def background_voice_listener(self):
        """A dedicated thread that listens for audio and processes it based on the current app state."""
        self._log("👂 Background voice listener started.")
        mic = sr.Microphone()
        with mic as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=0.5)

        while True:
            if self.state == AppState.FACE_DETECTED or self.state == AppState.LISTENING_FOR_FOLLOW_UP:
                self._log("🎤 Listening for speech...")
                try:
                    with mic as source:
                        # ### MODIFICATION START: Using the new time limit constant ###
                        audio = self.recognizer.listen(source, phrase_time_limit=PHRASE_TIME_LIMIT)
                        # ### MODIFICATION END ###
                    self._log("🎤 Audio captured, recognizing...")
                    text = self.recognizer.recognize_google(audio)
                    self._log(f"🎤 Recognition successful: '{text}'")
                    self.last_query = text
                    self.root.after(0, self.process_ai_query)
                except sr.UnknownValueError:
                    self._log("🎤 Recognition failed: Could not understand audio.")
                except Exception as e:
                    self._log(f"🎤 Listener error: {e}")
            
            time.sleep(0.1)

    def process_ai_query(self):
        """Handles the AI interaction using the hybrid logic."""
        # ... (this function remains the same) ...
        self.state = AppState.PROCESSING_AI
        self.spoken_text_label.config(text=f"You: {self.last_query}")
        self.response_label.config(text="Thinking...")
        self._send_eye_command("nodding")
        try:
            user_query_lower = self.last_query.lower()
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
                self._log(f"🔑 Keyword override! Forcing emotion to: '{emotion}'")
            elif len(parts) > 1:
                last_part = parts[-1].strip().lower()
                if last_part in ["happy", "sad", "angry", "pamper", "neutral"]:
                    emotion = last_part
                    self._log(f"🧠 AI classified its emotion as: '{emotion}'")
                else:
                    clean_text = "\n".join(parts).strip()
            self._log(f"🤖 AI says: {clean_text}")
            self.response_label.config(text=clean_text)
            self._send_eye_command(emotion)
            self.state = AppState.SPEAKING
            threading.Thread(target=self.speak_response, args=(clean_text,), daemon=True).start()
        except Exception as e:
            self._log(f"❌ Gemini Error: {e}")
            self.response_label.config(text="Sorry, an error occurred.")
            self.state = AppState.COOLDOWN
            self.state_timer = time.time()

    def speak_response(self, text):
        # ... (this function remains the same) ...
        self._log("🔊 TTS thread started.")
        try:
            self.tts_engine.say(text)
            self.tts_engine.runAndWait()
        finally:
            self._log("🔊 TTS finished. Switching to follow-up state.")
            self.state = AppState.LISTENING_FOR_FOLLOW_UP
            self.state_timer = time.time()

    def update_state_machine(self):
        # ... (this function remains the same) ...
        now = time.time()
        if self.state == AppState.IDLE:
            pass
        elif self.state == AppState.FACE_DETECTED:
            if now - self.state_timer > 5:
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
        elif self.state == AppState.COOLDOWN:
            self.spoken_text_label.config(text="Waiting for interaction...")
            if now - self.state_timer > FACE_DETECTION_COOLDOWN:
                self._log("⏱️ Cooldown finished. Returning to IDLE state.")
                self.state = AppState.IDLE
        self.update_video_frame()
        self.root.after(100, self.update_state_machine)
        
    def update_video_frame(self):
        # ... (this function remains the same) ...
        if not self.cap or not self.cap.isOpened():
            self.cap = cv2.VideoCapture(2)
            if not self.cap.isOpened():
                self._log("❌ Cannot access webcam")
                return
        ret, frame = self.cap.read()
        if ret:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
            if self.state == AppState.IDLE and len(faces) > 0:
                self._log("🙂 Face detected. Switching to listen for query.")
                self.state = AppState.FACE_DETECTED
                self.state_timer = time.time()
                self.spoken_text_label.config(text="Face detected. Ask a question.")
                self._send_eye_command("happy")
            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 90, 158), 3)
            frame = cv2.resize(frame, (640, 480))
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            imgtk = ImageTk.PhotoImage(image=Image.fromarray(rgb))
            self.video_label.imgtk = imgtk
            self.video_label.config(image=imgtk)

    def load_logo_from_url(self):
        # ... (this function remains the same) ...
        try:
            headers = {'User-Agent': 'Mozilla/5.0'}
            response = requests.get(TIST_LOGO_URL, headers=headers, stream=True)
            response.raise_for_status()
            logo_data = response.content
            logo_image = Image.open(io.BytesIO(logo_data))
            logo_image.thumbnail((250, 250))
            self.logo_photo = ImageTk.PhotoImage(logo_image)
            self.logo_label.config(image=self.logo_photo)
        except requests.exceptions.RequestException as e:
            self._log(f"❌ Could not download logo: {e}")
            self.logo_label.config(text="TIST", font=(FONT_FACE, 20, "bold"))

    def open_website(self):
        # ... (this function remains the same) ...
        webbrowser.open_new_tab(TIST_WEBSITE_URL)

    def on_closing(self):
        # ... (this function remains the same) ...
        self._log("🛑 Close button clicked. Shutting down.")
        if self.cap: self.cap.release()
        self.root.destroy()

if __name__ == "__main__":
    main_window = tk.Tk()
    app = AIAssistantApp(main_window)
    main_window.mainloop()
