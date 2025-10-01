import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from rclpy.timer import Timer

# The time in seconds to wait after the last person detection before stopping the interaction.
INTERACTION_TIMEOUT_SEC = 15.0

class InteractionManagerNode(Node):
    """
    Manages the robot's interaction state based on person detection.
    It controls when the robot should be listening for speech and what animations to show.
    """
    def __init__(self):
        super().__init__('interaction_manager')

        # State machine: 'idle' or 'interacting'
        self.state = 'idle'
        self.person_present = False
        self.interaction_timer: Timer = None

        # Subscribers
        self.person_sub = self.create_subscription(
            Bool,
            '/person_detected_status',
            self.person_callback,
            10)

        # Publishers
        self.robot_state_pub = self.create_publisher(String, '/robot_state', 10)
        self.emotion_pub = self.create_publisher(String, '/emotion', 10)

        self.get_logger().info('Interaction Manager started. Waiting for person detection.')

    def person_callback(self, msg: Bool):
        """Callback for when a person is detected or lost."""
        self.person_present = msg.data
        
        if self.person_present:
            # If a person is detected, always reset the interaction timer.
            self.reset_interaction_timer()

            # If we were previously idle, this is the start of a new interaction.
            if self.state == 'idle':
                self.start_interaction()
        
        # If a person is lost (msg.data is False), we do nothing immediately.
        # The on_interaction_timeout method will handle stopping the interaction.

    def start_interaction(self):
        """Begins an interaction sequence."""
        self.get_logger().info('Person detected. Starting new interaction.')
        self.state = 'interacting'
        
        # Show the "greet" animation once at the beginning.
        greet_msg = String()
        greet_msg.data = 'greet'
        self.emotion_pub.publish(greet_msg)

        # Tell the speech-to-text node to start listening.
        listen_msg = String()
        listen_msg.data = 'listening'
        self.robot_state_pub.publish(listen_msg)

    def reset_interaction_timer(self):
        """Resets the 15-second countdown timer."""
        # Cancel any existing timer to ensure we only have one running.
        if self.interaction_timer is not None and not self.interaction_timer.is_canceled():
            self.interaction_timer.cancel()
        
        # Create a new one-shot timer that will call on_interaction_timeout when it finishes.
        self.interaction_timer = self.create_timer(
            INTERACTION_TIMEOUT_SEC,
            self.on_interaction_timeout)
        self.get_logger().info(f'Interaction timer reset for {INTERACTION_TIMEOUT_SEC} seconds.')

    def on_interaction_timeout(self):
        """Called when 15 seconds have passed with no new person detection."""
        self.get_logger().info('Interaction timer expired.')
        
        # Cleanly destroy the timer object.
        if self.interaction_timer is not None:
            self.interaction_timer.destroy()
            self.interaction_timer = None

        # Only end the interaction if the timer expires AND the person is no longer present.
        if self.state == 'interacting' and not self.person_present:
            self.end_interaction()

    def end_interaction(self):
        """Ends the interaction sequence and returns to idle state."""
        self.get_logger().info('Person lost and timer expired. Ending interaction.')
        self.state = 'idle'

        # Tell the speech-to-text node to stop listening.
        # The node uses the "speaking" state as the command to stop the microphone.
        stop_listen_msg = String()
        stop_listen_msg.data = 'speaking' 
        self.robot_state_pub.publish(stop_listen_msg)

        # Set the eye animation back to neutral.
        neutral_msg = String()
        neutral_msg.data = 'neutral'
        self.emotion_pub.publish(neutral_msg)

def main(args=None):
    rclpy.init(args=args)
    interaction_manager_node = InteractionManagerNode()
    rclpy.spin(interaction_manager_node)
    interaction_manager_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

