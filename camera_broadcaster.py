import cv2
import os

# --- Configuration ---
REAL_CAMERA_INDEX = 0  # Your physical webcam
VIRTUAL_CAMERA_DEVICE = '/dev/video1' # The virtual device you created

def main():
    print("--- Camera Broadcaster ---")
    
    # Open the physical camera
    cap = cv2.VideoCapture(REAL_CAMERA_INDEX)
    if not cap.isOpened():
        print(f"Error: Could not open real camera at index {REAL_CAMERA_INDEX}")
        return

    # Get frame properties from the real camera
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    
    print(f"Real camera opened: {width}x{height} @ {fps}fps")

    # Open the virtual camera device for writing
    try:
        # Use FFMPEG to pipe frames to the v4l2 device
        # This is more reliable than trying to write directly with OpenCV
        ffmpeg_cmd = [
            'ffmpeg', '-y', '-f', 'rawvideo', 
            '-pixel_format', 'bgr24', # OpenCV uses BGR
            '-video_size', f'{width}x{height}', 
            '-framerate', str(fps), 
            '-i', '-', # Input from stdin
            '-f', 'v4l2', VIRTUAL_CAMERA_DEVICE
        ]
        import subprocess as sp
        pipe = sp.Popen(ffmpeg_cmd, stdin=sp.PIPE)
        print(f"Streaming to virtual camera: {VIRTUAL_CAMERA_DEVICE}")
    except Exception as e:
        print(f"Error: Could not open virtual camera pipe. {e}")
        cap.release()
        return

    print("Broadcasting... Press Ctrl+C in this terminal to stop.")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            # Write the frame to the ffmpeg pipe
            pipe.stdin.write(frame.tobytes())
            
    except KeyboardInterrupt:
        print("\nStopping broadcaster.")
    finally:
        # Clean up
        pipe.stdin.close()
        pipe.wait()
        cap.release()
        print("Broadcast finished.")

if __name__ == '__main__':
    main()