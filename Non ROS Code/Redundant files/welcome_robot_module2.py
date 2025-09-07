import cv2
import threading
import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk
import speech_recognition as sr
import requests
import pyttsx3

API_KEY = "sk-94b2320f54ca43f588ca7f7daa4e8848"

def get_deepseek_response(user_input):
    url = "https://api.deepseek.com/v1/chat/completions"
    headers = {
        "Authorization": f"Bearer {API_KEY}",
        "Content-Type": "application/json"
    }
    payload = {
        "model": "deepseek-chat",
        "messages": [
            {"role": "system", "content": "You are a helpful robot at an educational institute."},
            {"role": "user", "content": user_input}
        ]
    }
    try:
        res = requests.post(url, headers=headers, json=payload, timeout=10)
        res.raise_for_status()
        return res.json()["choices"][0]["message"]["content"]
    except Exception as e:
        print(f"❌ DeepSeek Error: {e}")
        return "Sorry, I could not get a response right now."

# Load OpenCV face detection model
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
ai_triggered = False  # To avoid multiple triggers

# --- Function to trigger AI ---
def trigger_ai(source, query=None):
    global ai_triggered
    if not ai_triggered:
        ai_triggered = True
        print(f"✅ AI triggered via: {source}")
        if query:
            print(f"🗨️ User said: {query}")
            # 🔗 Get AI response
            response = get_deepseek_response(query)
            print(f"🤖 AI says: {response}")
            # 🖼️ Show in GUI
            response_label.config(text=f"🤖 {response}")
            # 🔊 Speak out loud
            speak_response(response)
        else:
            messagebox.showwarning("No Input", "No question to answer.")


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
            messagebox.showerror("Error", "Sorry, could not understand audio.")
        except sr.RequestError:
            messagebox.showerror("Error", "Speech recognition service failed.")
        return None


# --- Webcam detection loop ---
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

        # Trigger if face is detected
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            query = get_voice_input()
            trigger_ai("Face Detection", query)
            break


        # Display in GUI
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(rgb)
        imgtk = ImageTk.PhotoImage(image=img)
        video_label.imgtk = imgtk
        video_label.configure(image=imgtk)

    cap.release()

# --- Handle manual button click ---
def on_button_click():
    user_input = input_box.get().strip()
    if user_input:
        trigger_ai("Button + Text", user_input)
    else:
        messagebox.showwarning("Input Missing", "Please enter a question.")

# --- Handle voice input ---
def on_voice_input():
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        print("🎙️ Listening... Please speak.")
        messagebox.showinfo("Voice Input", "Listening... Please speak.")
        try:
            audio = recognizer.listen(source)  # removed timeout
            query = recognizer.recognize_google(audio)
            print(f"🗨️ Recognized: {query}")
            spoken_text_label.config(text=f"You said: {query}")
            trigger_ai("Voice Input", query)
        except sr.UnknownValueError:
            messagebox.showerror("Error", "Sorry, could not understand audio.")
        except sr.RequestError:
            messagebox.showerror("Error", "Speech recognition service failed.")

def speak_response(text):
    engine = pyttsx3.init()
    engine.say(text)
    engine.runAndWait()

# ---------------- GUI Setup ----------------
window = tk.Tk()
window.title("Welcome Robot – Module 2")

# Webcam display
video_label = tk.Label(window)
video_label.pack()
spoken_text_label = tk.Label(window, text="", font=("Arial", 12), fg="blue")
spoken_text_label.pack(pady=5)
response_label = tk.Label(
    window,
    text="",
    font=("Arial", 13),
    wraplength=500,
    justify="center",
    fg="green"
)
response_label.pack(pady=10)
# Text input box
input_box = tk.Entry(window, font=("Arial", 14), width=40)
input_box.pack(pady=5)

# Buttons
text_button = tk.Button(window, text="Submit Text", font=("Arial", 12), command=on_button_click)
text_button.pack(pady=5)

voice_button = tk.Button(window, text="Speak Now", font=("Arial", 12), command=on_voice_input)
voice_button.pack(pady=5)

# Start face detection
threading.Thread(target=start_face_detection, daemon=True).start()
window.mainloop()
