import cv2
import threading
import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk
import speech_recognition as sr

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
        messagebox.showinfo("AI Started", f"AI triggered via: {source}\nQuery: {query or 'No input'}")
        # In Module 3, send query to DeepSeek here

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
            trigger_ai("Face Detection")
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

# ---------------- GUI Setup ----------------
window = tk.Tk()
window.title("Welcome Robot – Module 2")

# Webcam display
video_label = tk.Label(window)
video_label.pack()
spoken_text_label = tk.Label(window, text="", font=("Arial", 12), fg="blue")
spoken_text_label.pack(pady=5)

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
