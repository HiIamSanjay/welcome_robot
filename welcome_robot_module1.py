import cv2
import threading
import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk

# Load OpenCV face detection model
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

ai_triggered = False  # To avoid triggering multiple times

# --- Function to trigger AI ---
def trigger_ai(source):
    global ai_triggered
    if not ai_triggered:
        ai_triggered = True
        print(f"✅ AI triggered via: {source}")
        messagebox.showinfo("AI Started", f"AI triggered via: {source}")
        # Future: Send user input to DeepSeek API here

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

# --- Button click handler ---
def on_button_click():
    trigger_ai("Button Press")

# --- GUI setup ---
window = tk.Tk()
window.title("Welcome Robot – Module 1")

video_label = tk.Label(window)
video_label.pack()

start_button = tk.Button(window, text="Start AI", font=("Arial", 14), command=on_button_click)
start_button.pack(pady=10)

# Start webcam in separate thread
threading.Thread(target=start_face_detection, daemon=True).start()

# Launch GUI loop
window.mainloop()
