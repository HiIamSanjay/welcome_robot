import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
from ctypes import CFUNCTYPE, c_char_p, c_int, cdll
import threading

# --- Configuration ---
# Reduced pause threshold for faster phrase detection
PAUSE_THRESHOLD = 0.5

# --- Helper Functions ---
def find_mic_by_name(target_name: str) -> int:
    """
    Finds the device index for a microphone by its name (case-insensitive).
    Returns the integer index or None if not found.
    """
    mic_list = sr.Microphone.list_microphone_names()
    for i, mic_name in enumerate(mic_list):
        if target_name.lower() in mic_name.lower():
            return i
    return None

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

class SpeechToTextNode(Node):
    def __init__(self):
        super().__init__('speech_to_text_node')
        self.publisher_ = self.create_publisher(String, '/transcribed_text', 10)
        
        self.state_subscriber = self.create_subscription(
            String,
            '/robot_state',
            self.state_callback,
            10)
        
        self.recognizer = sr.Recognizer()
        self.recognizer.pause_threshold = PAUSE_THRESHOLD
        # --- KEY CHANGE: Explicitly set non_speaking_duration ---
        # This ensures the recognizer finalizes a phrase after 0.6s of silence.
        self.recognizer.non_speaking_duration = PAUSE_THRESHOLD
        self.listener_lock = threading.Lock()

        mic_name = "Audio Array"
        self.get_logger().info(f"Searching for microphone named '{mic_name}'...")
        self.mic_index = find_mic_by_name(mic_name)
        
        if self.mic_index is None:
            self.get_logger().error(f"Could not find microphone named '{mic_name}'. Falling back to default index.")
        else:
            self.get_logger().info(f"Found '{mic_name}' at device index {self.mic_index}.")

        try:
            with sr.Microphone(device_index=self.mic_index) as source:
                self.get_logger().info("Calibrating microphone to ambient noise...")
                self.recognizer.adjust_for_ambient_noise(source, duration=2.0)
                self.recognizer.dynamic_energy_threshold = True
                self.get_logger().info("Calibration complete. Dynamic energy threshold enabled.")
        except Exception as e:
            self.get_logger().error(f"Could not open microphone for calibration: {e}")
            return

        self.listener_handle = None
        self.get_logger().info("Speech-to-Text node initialized.")

    def state_callback(self, msg):
        with self.listener_lock:
            if msg.data == "listening" and self.listener_handle is None:
                self.get_logger().info("State: LISTENING. Starting background listener...")
                try:
                    source = sr.Microphone(device_index=self.mic_index)
                    stop_function = self.recognizer.listen_in_background(
                        source, self.recognition_callback
                    )
                    self.listener_handle = (stop_function, source)
                except Exception as e:
                    self.get_logger().error(f"Failed to start listener: {e}")

            elif msg.data == "speaking" and self.listener_handle is not None:
                self.get_logger().info("State: SPEAKING. Stopping background listener...")
                try:
                    stop_function, _ = self.listener_handle
                    stop_function(wait_for_stop=True)
                except Exception as e:
                    self.get_logger().error(f"Failed to stop listener cleanly: {e}")
                finally:
                    self.listener_handle = None

    def recognition_callback(self, recognizer, audio_data):
        self.get_logger().info("Phrase detected, preparing for recognition...")
        threading.Thread(target=self.process_audio, args=(audio_data,), daemon=True).start()

    def process_audio(self, audio_data):
        try:
            self.get_logger().info("Recognizing...")
            text = self.recognizer.recognize_google(audio_data)
            self.get_logger().info(f'Successfully transcribed: "{text}"')
            
            msg = String()
            msg.data = text
            self.publisher_.publish(msg)

        except sr.UnknownValueError:
            self.get_logger().warn("Google could not understand the audio.")
        except sr.RequestError as e:
            self.get_logger().error(f"Google API error; {e}")
        except Exception as e:
            self.get_logger().error(f'An unexpected error occurred during recognition: {e}')

def main(args=None):
    suppress_alsa_errors()
    rclpy.init(args=args)
    speech_to_text_node = SpeechToTextNode()
    rclpy.spin(speech_to_text_node)
    speech_to_text_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


