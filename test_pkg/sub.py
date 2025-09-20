# Import rclpy and Node base class
import rclpy
from rclpy.node import Node
# Import the same message type as the publisher (String)
from std_msgs.msg import String


# Define a class Listener that inherits from Node
class sub(Node):
    def __init__(self):
        # Initialize the node with the name "listener"
        super().__init__("sub")

        # Create a subscriber that listens on the "chatter" topic
        # It expects String messages, queue size = 10
        self.subscription = self.create_subscription(
            String,          # message type
            "chatter",       # topic name (must match publisher)
            self.callback,   # function to call when a message arrives
            10               # queue size
        )

    # Callback function that runs every time a new message is received
    def callback(self, msg):
        # Print the message contents
        self.get_logger().info(f"I heard: {msg.data}")


def main():
    # Initialize ROS 2
    rclpy.init()

    # Create an instance of the sub node
    node = sub()

    # Spin keeps the node alive and processing callbacks
    rclpy.spin(node)

    # Cleanup after shutdown
    node.destroy_node()
    rclpy.shutdown()


# Standard Python entry point
if __name__ == "__main__":
    main()
