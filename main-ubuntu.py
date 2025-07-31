import cv2
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
import speech_recognition as sr
import google.generativeai as genai
import pyttsx3  # MODIFIED: Using pyttsx3 for TTS
import pygame
import io
import time
import webbrowser
import requests

# --- Configuration Constants ---
GEMINI_API_KEY = "AIzaSyDv1KDhOOfv7lH3MT-hJJ8r2SD9oZ4_NXY"

# --- Toc H Themed Constants ---
BG_COLOR = "#0A2239"
TEXT_COLOR = "#FFFFFF"
ACCENT_COLOR = "#005a9e"
FONT_FACE = "Segoe UI"
FOLLOW_UP_TIMEOUT = 10
TIST_WEBSITE_URL = "https://tistcochin.edu.in/"
TIST_LOGO_URL = "https://tistcochin.edu.in/wp-content/uploads/2022/08/TISTlog-trans.png"

# --- AI Model Constants ---
GEMINI_MODEL = "gemini-2.5-flash" # Corrected model name
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

GEMINI_PROMPT = f"""You are a multi-faceted Robot brain, the brain of a physical robot. Your primary goal is to determine the user's intent and respond in the correct persona.
**Step 1: Analyze the User's Query**
Classify the query into one of three categories:
1.  **Personal/Sentimental Query:** A question directed at you (e.g., "How are you?", "Are you a robot?", "Where are you?").
2.  **TIST-Specific Query:** A question about Toc H Institute of Science and Technology (TIST).
3.  **General Knowledge Query:** Any other question.

**Step 2: Follow the Response Protocol**

* **If Personal/Sentimental:**
    * **Action:** Respond as a friendly, helpful robot. Use "I" and "me".
    * **Tone:** Natural and friendly.
    * **Constraint:** For most personal questions, DO NOT mention TIST college.
    * **Exception:** If asked about your physical location (e.g., "where are you?"), you MUST state your location at the college. For example: "I am right here with you at the Toc H Institute of Science and Technology campus."
    * **Example (Non-Location):** If asked "are you a robot?", say "Yes, I am! I'm a robot designed to help you."

* **If TIST-Specific:**
    * **Action:** First, try to answer using ONLY the provided TIST context below. If the answer isn't in the context, use your general knowledge to find information *specifically about Toc H Institute of Science and Technology* and answer professionally.
    * **Persona:** Act as a professional, formal representative of TIST.
    * **Formatting:** All responses must be plain text. No emojis, markdown, or conversational fillers.

* **If General Knowledge:**
    * **Action:** Answer the question directly and factually.
    * **Tone:** Neutral and informative.

**TIST Context for TIST-Specific Queries:**
---
{COLLEGE_CONTEXT}
---
"""

class AIAssistantApp:
    def __init__(self, root):
        self.root = root
        self.text_input_visible = False
        self.ai_triggered = threading.Event()
        self.cap = None

        if not GEMINI_API_KEY or "YOUR_API_KEY" in GEMINI_API_KEY:
            messagebox.showerror("API Key Error", "Please paste your Gemini API key into the script.")
            root.destroy()
            return
        genai.configure(api_key=GEMINI_API_KEY)

        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        self.recognizer = sr.Recognizer()
        self.gemini_model = genai.GenerativeModel(model_name=GEMINI_MODEL)

        # MODIFIED: Initialize the pyttsx3 engine
        self.tts_engine = pyttsx3.init()
        
        self.setup_ui()
        threading.Thread(target=self.load_logo_from_url, daemon=True).start()
        threading.Thread(target=self.update_video_feed, daemon=True).start()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

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
        if self.text_input_visible:
            self.text_input_frame.pack_forget()
            self.text_input_visible = False
        else:
            self.text_input_frame.pack(side="top", fill="x", pady=5)
            self.input_box.focus_set()
            self.text_input_visible = True

    def load_logo_from_url(self):
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
            print(f"❌ Could not download logo: {e}")
            self.logo_label.config(text="TIST", font=(FONT_FACE, 20, "bold"))

    def open_website(self):
        webbrowser.open_new_tab(TIST_WEBSITE_URL)

    def on_closing(self):
        if self.cap: self.cap.release()
        self.root.destroy()
        
    def speak_response(self, text):
        # MODIFIED: This function now uses the pyttsx3 engine
        def _play_with_pyttsx3():
            try:
                self.tts_engine.say(text)
                self.tts_engine.runAndWait()
            except Exception as e:
                print(f"❌ Audio Error: {e}")
            finally:
                self.listen_for_follow_up()
        threading.Thread(target=_play_with_pyttsx3, daemon=True).start()

    def listen_for_follow_up(self):
        threading.Thread(target=self._handle_follow_up, daemon=True).start()

    def _handle_follow_up(self):
        query = self.get_voice_input(
            prompt="I'm listening for a follow-up...", 
            listen_timeout=FOLLOW_UP_TIMEOUT
        )
        if query:
            self.root.after(0, lambda: self.trigger_ai("Follow-up", query))
        else:
            print("🕒 Follow-up timeout. Reverting to face detection.")
            self.spoken_text_label.config(text="Waiting for interaction...")
            self.ai_triggered.clear()

    def get_gemini_response(self, user_input):
        try:
            full_prompt = f"{GEMINI_PROMPT}\n\nUser Query: \"{user_input}\""
            response = self.gemini_model.generate_content(full_prompt)
            return response.text
        except Exception as e:
            print(f"❌ Gemini Error: {e}")
            return "Sorry, there was an error connecting to my services."

    def trigger_ai(self, source, query=None):
        if self.ai_triggered.is_set() and source != "Follow-up": return
        self.ai_triggered.set()
        print(f"✅ AI triggered via: {source}\n🗨️ User said: {query}")
        self.spoken_text_label.config(text=f"You: {query}")
        self.response_label.config(text="Thinking...")
        response_text = self.get_gemini_response(query)
        clean_text = response_text.replace('*', '').strip()
        print(f"🤖 AI says: {clean_text}")
        self.response_label.config(text=f"{clean_text}")
        self.speak_response(clean_text)

    def get_voice_input(self, prompt="Listening...", listen_timeout=5):
        with sr.Microphone() as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
            self.spoken_text_label.config(text=f"🎙️ {prompt}")
            self.root.update_idletasks()
            try:
                audio = self.recognizer.listen(source, timeout=listen_timeout, phrase_time_limit=10)
                return self.recognizer.recognize_google(audio)
            except sr.WaitTimeoutError:
                return None
            except sr.UnknownValueError:
                self.spoken_text_label.config(text="Could not understand audio. Try again.")
                return None
            except sr.RequestError:
                messagebox.showerror("Error", "Speech recognition service is unavailable.")
                return None

    def update_video_feed(self):
        self.cap = cv2.VideoCapture(0)
        while self.root.winfo_exists():
            ret, frame = self.cap.read()
            if not ret: break
            
            if not self.ai_triggered.is_set():
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
                for (x, y, w, h) in faces:
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 90, 158), 3)
                    if not self.ai_triggered.is_set():
                        query = self.get_voice_input(prompt="Face detected. Ask a question.")
                        if query: self.trigger_ai("Face Detection", query)
                        break
            
            if ret:
                frame = cv2.resize(frame, (640, 480))
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                imgtk = ImageTk.PhotoImage(image=Image.fromarray(rgb))
                self.video_label.imgtk = imgtk
                self.video_label.config(image=imgtk)
        if self.cap: self.cap.release()

    def on_submit_text(self):
        user_input = self.input_box.get().strip()
        if user_input:
            self.input_box.delete(0, tk.END)
            if self.text_input_visible:
                self.toggle_text_input()
            self.trigger_ai("Text Input", user_input)
        else:
            messagebox.showwarning("Input Missing", "Please enter a question.")

    def on_start_voice_input(self):
        if self.ai_triggered.is_set():
            messagebox.showinfo("Busy", "The assistant is currently speaking.")
            return
        query = self.get_voice_input()
        if query: self.trigger_ai("Voice Button", query)


if __name__ == "__main__":
    main_window = tk.Tk()
    app = AIAssistantApp(main_window)
    main_window.mainloop()
