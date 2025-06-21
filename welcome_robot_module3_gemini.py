import cv2
import threading
import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk
import speech_recognition as sr
import pyttsx3
import google.generativeai as genai

# --- Gemini API Key ---
GEMINI_API_KEY = "AIzaSyBg_pr2pOdcPRuY1odTjAkS7Hs_gQ2EYyo"
genai.configure(api_key=GEMINI_API_KEY)

# Initialize TTS engine
engine = pyttsx3.init()

# Load face detection model
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
ai_triggered = False

from gtts import gTTS
import pygame
import tempfile
import os

def speak_response(text):
    try:
        # Generate speech
        tts = gTTS(text=text, lang='en')
        with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as fp:
            temp_path = fp.name
            tts.save(temp_path)

        # Initialize pygame mixer
        pygame.mixer.init()
        pygame.mixer.music.load(temp_path)
        pygame.mixer.music.play()

        # Wait for playback to finish
        while pygame.mixer.music.get_busy():
            continue

        pygame.mixer.music.unload()
        os.remove(temp_path)
    except Exception as e:
        print(f"❌ TTS Error: {e}")

# --- Call Gemini AI ---
def get_gemini_response(user_input):
    try:
        model = genai.GenerativeModel(model_name="gemini-2.0-flash")
        response = model.generate_content(
            [f"You are a helpful robot at an educational institute. {user_input}"]
        )
        return response.text
    except Exception as e:
        print(f"❌ Gemini Error: {e}")
        return "Sorry, I couldn't get a response right now."

# --- AI Trigger Function ---
def trigger_ai(source, query=None):
    global ai_triggered
    if not ai_triggered:
        ai_triggered = True
        print(f"✅ AI triggered via: {source}")
        if query:
            print(f"🗨️ User said: {query}")
            response = get_gemini_response(query)
            print(f"🤖 AI says: {response}")
            response_label.config(text=f"🤖 {response}")
            speak_response(response)
        else:
            messagebox.showwarning("No Input", "No question received.")

def get_voice_input():
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        print("🎙️ Listening after face detection...")
        try:
            audio = recognizer.listen(source)
            query = recognizer.recognize_google(audio)
            print(f"🗨️ Recognized: {query}")
            spoken_text_label.config(text=f"You said: {query}")
            return query
        except sr.UnknownValueError:
            messagebox.showerror("Error", "Could not understand audio.")
        except sr.RequestError:
            messagebox.showerror("Error", "Speech recognition service failed.")
        return None

# --- Webcam Thread ---
def start_face_detection():
    cap = cv2.VideoCapture(0)
    while True:
        if ai_triggered:
            break
        ret, frame = cap.read()
        if not ret:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 5)

        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            query = get_voice_input()
            trigger_ai("Face Detection", query)
            break

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(rgb)
        imgtk = ImageTk.PhotoImage(image=img)
        video_label.imgtk = imgtk
        video_label.configure(image=imgtk)

    cap.release()

# --- Text button ---
def on_button_click():
    user_input = input_box.get().strip()
    if user_input:
        trigger_ai("Button + Text", user_input)
    else:
        messagebox.showwarning("Input Missing", "Please enter a question.")

# --- Voice button ---
def on_voice_input():
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        print("🎙️ Listening... Please speak.")
        messagebox.showinfo("Voice Input", "Listening... Please speak.")
        try:
            audio = recognizer.listen(source)
            query = recognizer.recognize_google(audio)
            print(f"🗨️ Recognized: {query}")
            spoken_text_label.config(text=f"You said: {query}")
            trigger_ai("Voice Input", query)
        except sr.UnknownValueError:
            messagebox.showerror("Error", "Could not understand audio.")
        except sr.RequestError:
            messagebox.showerror("Error", "Speech recognition service failed.")

# --- GUI Setup ---
window = tk.Tk()
window.title("Welcome Robot – Gemini Version")

video_label = tk.Label(window)
video_label.pack()

spoken_text_label = tk.Label(window, text="", font=("Arial", 12), fg="blue")
spoken_text_label.pack(pady=5)

response_label = tk.Label(window, text="", font=("Arial", 13), wraplength=500, justify="center", fg="green")
response_label.pack(pady=10)

input_box = tk.Entry(window, font=("Arial", 14), width=40)
input_box.pack(pady=5)

text_button = tk.Button(window, text="Submit Text", font=("Arial", 12), command=on_button_click)
text_button.pack(pady=5)

voice_button = tk.Button(window, text="Speak Now", font=("Arial", 12), command=on_voice_input)
voice_button.pack(pady=5)

threading.Thread(target=start_face_detection, daemon=True).start()
window.mainloop()
