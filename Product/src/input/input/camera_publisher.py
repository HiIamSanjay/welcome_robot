import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time

class CameraPublisher(Node):
    """
    This node captures video from a webcam and publishes it as ROS Image messages.
    It now automatically finds the first available camera.
    """
    def __init__(self):
        super().__init__('camera_publisher_node')
        
        # Publisher for the video feed
        self.publisher_ = self.create_publisher(Image, '/video_feed', 10)
        
        # Timer to publish frames at a regular interval (e.g., 30 FPS)
        timer_period = 1.0 / 30.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # --- KEY CHANGE: Automatic Camera Detection ---
        self.cap = None
        # Loop through a range of possible camera indices (e.g., 0 to 10)
        for i in range(10):
            self.get_logger().info(f"Checking for camera at index {i}...")
            cap_test = cv2.VideoCapture(i)
            # Give it a moment to initialize
            time.sleep(0.2)
            if cap_test.isOpened():
                self.get_logger().info(f"✅ Success! Camera found and opened at index {i}.")
                self.cap = cap_test
                break # Exit the loop once a camera is found
            else:
                cap_test.release()
        
        # If no camera was found after checking all indices, log an error and shut down.
        if self.cap is None:
            self.get_logger().error("❌ Could not find any available video capture devices.")
            rclpy.shutdown()
            return

        self.bridge = CvBridge()
        self.get_logger().info("Camera Publisher node started.")

    def timer_callback(self):
        """
        Called by the timer to capture and publish a single video frame.
        """
        if self.cap is None:
            return
            
        ret, frame = self.cap.read()
        
        if ret:
            try:
                # Convert the OpenCV image to a ROS Image message and publish it
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.publisher_.publish(ros_image)
            except CvBridgeError as e:
                self.get_logger().error(f'Failed to convert and publish frame: {e}')
        else:
            self.get_logger().warn("Could not read frame from camera. It might be disconnected.")


    def destroy_node(self):
        """Cleanly release the camera resource when the node is shut down."""
        if self.cap:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    # Only create and spin the node if rclpy is not already shut down
    if rclpy.ok():
        camera_publisher = CameraPublisher()
        if rclpy.ok(): # Check again in case camera init failed
            try:
                rclpy.spin(camera_publisher)
            except KeyboardInterrupt:
                pass
            finally:
                camera_publisher.destroy_node()
                rclpy.shutdown()

if __name__ == '__main__':
    main()


