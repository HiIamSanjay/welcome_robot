import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
from ctypes import CFUNCTYPE, c_char_p, c_int, cdll
import threading

# --- Configuration ---
MANUAL_ENERGY_THRESHOLD = 4000
PAUSE_THRESHOLD = 2.0
PHRASE_TIME_LIMIT = 15

# --- Helper Functions ---
def find_mic_by_name(logger, target_name: str) -> int | None:
    """
    Finds the device index for a microphone by its name, case-insensitively.
    
    Args:
        logger: The ROS 2 node logger for output.
        target_name: The name of the microphone to search for (e.g., "Audio array").

    Returns:
        The integer index of the microphone, or None if not found.
    """
    try:
        mic_list = sr.Microphone.list_microphone_names()
        logger.info("Available microphones: " + str(mic_list))
        for i, mic_name in enumerate(mic_list):
            # --- MODIFICATION: Make the search case-insensitive ---
            if target_name.lower() in mic_name.lower():
                return i
    except Exception as e:
        logger.error(f"Could not list microphones: {e}")
    return None

def py_error_handler(filename, line, function, err, fmt):
    pass

ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)

def suppress_alsa_errors():
    """Suppresses ALSA error messages from being printed to the console."""
    try:
        asound = cdll.LoadLibrary('libasound.so.2')
        asound.snd_lib_error_set_handler(c_error_handler)
    except (OSError, AttributeError):
        pass # Library not found, or handler setting failed.

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
        self.recognizer.energy_threshold = MANUAL_ENERGY_THRESHOLD
        self.recognizer.pause_threshold = PAUSE_THRESHOLD
        
        # Auto-detect microphone
        mic_name_to_find = "Audio Array"
        self.get_logger().info(f"Searching for microphone named '{mic_name_to_find}'...")
        
        mic_index = find_mic_by_name(self.get_logger(), mic_name_to_find)
        
        if mic_index is None:
            self.get_logger().error(f"Could not find microphone '{mic_name_to_find}'.")
            self.get_logger().warn("Falling back to the system's default microphone.")
            self.microphone = sr.Microphone()
        else:
            self.get_logger().info(f"Found '{mic_name_to_find}' at device index {mic_index}.")
            self.microphone = sr.Microphone(device_index=mic_index)
        
        self.stop_listening_function = None
        self.get_logger().info("Speech-to-Text node initialized.")

    def state_callback(self, msg):
        """Starts or stops the background listener based on the robot's state."""
        if msg.data == "listening" and self.stop_listening_function is None:
            self.get_logger().info("State: LISTENING. Starting background listener...")
            self.stop_listening_function = self.recognizer.listen_in_background(
                self.microphone, self.recognition_callback, phrase_time_limit=PHRASE_TIME_LIMIT
            )
        elif msg.data == "speaking" and self.stop_listening_function is not None:
            self.get_logger().info("State: SPEAKING. Stopping background listener...")
            self.stop_listening_function(wait_for_stop=False)
            self.stop_listening_function = None

    def recognition_callback(self, recognizer, audio_data):
        self.get_logger().info("Phrase detected, preparing for recognition...")
        threading.Thread(target=self.process_audio, args=(audio_data,), daemon=True).start()

    def process_audio(self, audio_data):
        """Performs speech recognition and publishes the result."""
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


