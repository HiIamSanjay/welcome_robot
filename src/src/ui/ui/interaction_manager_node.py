import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from rclpy.timer import Timer

class InteractionManagerNode(Node):
    """
    Manages the overall state of the robot's interaction with a user.
    - Activates the microphone when a person is detected.
    - Manages a grace period timer to keep the interaction alive if the person briefly disappears.
    - Centralizes the logic for switching between 'listening' and 'speaking' states.
    """
    def __init__(self):
        super().__init__('interaction_manager_node')

        # --- Subscribers ---
        self.person_sub = self.create_subscription(
            Bool, '/person_detected_status', self.person_callback, 10)
        self.ai_response_sub = self.create_subscription(
            String, '/ai_response', self.ai_response_callback, 10)
        self.finished_speaking_sub = self.create_subscription(
            String, '/finished_speaking', self.finished_speaking_callback, 10)

        # --- Publisher ---
        self.state_publisher = self.create_publisher(String, '/robot_state', 10)

        # --- State Management ---
        self.is_active = False # Is there an ongoing interaction session?
        self.is_person_present = False # Is a person currently detected?
        self.grace_period_timer: Timer = None
        self.interaction_timeout = 15.0 # Seconds

        self.get_logger().info("Interaction Manager started with improved state logic.")

    def person_callback(self, msg):
        """Handles changes in person detection."""
        person_now_present = msg.data

        if person_now_present and not self.is_person_present: # Person just appeared
            self.get_logger().info("Person appeared.")
            if self.grace_period_timer:
                self.grace_period_timer.cancel()
                self.grace_period_timer = None
                self.get_logger().info("Grace period timer cancelled.")
            if not self.is_active:
                self.start_interaction()

        elif not person_now_present and self.is_person_present: # Person just disappeared
            self.get_logger().info("Person disappeared.")
            if self.is_active:
                self.get_logger().info(f"Starting {self.interaction_timeout}s grace period.")
                if self.grace_period_timer: self.grace_period_timer.cancel()
                self.grace_period_timer = self.create_timer(self.interaction_timeout, self.end_interaction)
        
        self.is_person_present = person_now_present

    def start_interaction(self):
        """Begins a new conversation session."""
        self.get_logger().info("Interaction started.")
        self.is_active = True
        self.set_robot_state("listening")

    def end_interaction(self):
        """Ends the conversation session after a timeout."""
        self.get_logger().info("Interaction ended due to timeout.")
        self.is_active = False
        if self.grace_period_timer:
            self.grace_period_timer.cancel()
            self.grace_period_timer = None
        # No need to set state; the system will just go quiet.

    def ai_response_callback(self, msg):
        """Sets the state to 'speaking' when the AI provides a response."""
        if self.is_active:
            self.get_logger().info("AI responded. Setting state to SPEAKING.")
            self.set_robot_state("speaking")
            # If the grace period timer was running, cancel it.
            if self.grace_period_timer:
                self.grace_period_timer.cancel()
                self.grace_period_timer = None

    def finished_speaking_callback(self, msg):
        """Returns the robot to the 'listening' state after it has spoken."""
        self.get_logger().info("GUI finished speaking.")
        # --- KEY CHANGE ---
        # If the interaction is still active, always return to listening.
        # The grace period timer will handle ending the interaction if the person is gone.
        if self.is_active:
            self.get_logger().info("Returning to LISTENING state.")
            self.set_robot_state("listening")

    def set_robot_state(self, state: str):
        """Publishes a new state to the /robot_state topic."""
        msg = String()
        msg.data = state
        self.state_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    interaction_manager_node = InteractionManagerNode()
    rclpy.spin(interaction_manager_node)
    interaction_manager_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


