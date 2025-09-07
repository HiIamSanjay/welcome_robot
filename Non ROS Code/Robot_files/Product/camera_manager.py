import cv2
import pyfakewebcam
import time

# --- Configuration ---
REAL_CAMERA_INDEX = 0      # Usually 0 for the built-in webcam
WIDTH, HEIGHT = 640, 480

# ❗ Define the two virtual camera devices you created
VIRTUAL_CAMERA_GUI = '/dev/video2'
VIRTUAL_CAMERA_APP = '/dev/video3'


def start_camera_multiplexer():
    """
    Reads from a single real camera and streams the same feed to two
    separate virtual cameras simultaneously.
    """
    # Initialize the real camera
    real_cam = cv2.VideoCapture(REAL_CAMERA_INDEX)
    if not real_cam.isOpened():
        print(f"❌ ERROR: Cannot open real camera at index {REAL_CAMERA_INDEX}")
        return
        
    real_cam.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    real_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    print(f"✅ Real camera opened at index {REAL_CAMERA_INDEX}")

    # Initialize the two virtual cameras
    try:
        fake_cam_gui = pyfakewebcam.FakeWebcam(VIRTUAL_CAMERA_GUI, WIDTH, HEIGHT)
        print(f"✅ Virtual camera for GUI ready at {VIRTUAL_CAMERA_GUI}")
        
        fake_cam_app = pyfakewebcam.FakeWebcam(VIRTUAL_CAMERA_APP, WIDTH, HEIGHT)
        print(f"✅ Virtual camera for App ready at {VIRTUAL_CAMERA_APP}")
    except Exception as e:
        print(f"❌ ERROR: Could not create virtual cameras. Is v4l2loopback loaded with devices=2? Error: {e}")
        real_cam.release()
        return

    print("🚀 Camera multiplexer is running. Streaming to both virtual cameras...")
    print("   (You can now run your GUI and robot server scripts)")

    while True:
        success, frame = real_cam.read()
        if not success:
            print("⚠️ Lost connection to real camera. Retrying...")
            time.sleep(1)
            continue

        # Convert the frame to RGB once
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Send the same frame to both virtual cameras
        fake_cam_gui.schedule_frame(rgb_frame)
        fake_cam_app.schedule_frame(rgb_frame)
        
        # Control the frame rate
        time.sleep(1/30) # Stream at ~30 FPS

if __name__ == "__main__":
    start_camera_multiplexer()