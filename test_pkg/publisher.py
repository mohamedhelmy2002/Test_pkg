# Import rclpy, the core client library for ROS 2 in Python
import rclpy
# Import the Node class (all ROS 2 nodes must inherit from this)
from rclpy.node import Node
# Import the standard ROS 2 message type "String"
from std_msgs.msg import String


# Define a class Talker that inherits from rclpy's Node
class Talker(Node):
    def __init__(self):
        # Initialize the parent class (Node) with the name "talker"
        super().__init__("talker")

        # Create a publisher that publishes String messages
        # Topic name = "chatter", Queue size = 10
        self.publisher = self.create_publisher(String, "chatter", 10)

        # A simple counter variable to keep track of how many messages were sent
        self.counter = 0

        # Create a timer that calls self.timer_callback every 1.0 seconds
        self.timer = self.create_timer(.2, self.timer_callback)

    # This function runs every time the timer triggers
    def timer_callback(self):
        # Create a new String message
        msg = String()
        # Fill the message with text + the current counter value
        msg.data = f"Hello ROS2: {self.counter}"
        # Publish the message on the "chatter" topic
        self.publisher.publish(msg)
        # AS PRINT 
        self.get_logger().info(f'Publishing: "{msg.data}"')  

        # Increment the counter for the next message
        self.counter += 1


def main():
    # Initialize ROS 2 communication
    rclpy.init()

    # Create an instance of the Talker node
    node = Talker()

    # Spin keeps the node running (processing callbacks, timers, etc.)
    rclpy.spin(node)

    # Cleanup: destroy the node after spinning is done
    node.destroy_node()

    # Shutdown ROS 2 communication
    rclpy.shutdown()


# Standard Python entry point check
# Ensures main() only runs if this file is executed directly
if __name__ == "__main__":
    main()
