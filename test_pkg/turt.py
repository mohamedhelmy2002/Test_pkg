# Import rclpy, the core client library for ROS 2 in Python
import rclpy
# Import the Node class (all ROS 2 nodes must inherit from this)
from rclpy.node import Node
# Import the standard ROS 2 message type "String"
from geometry_msgs.msg import Twist


# Define a class Talker that inherits from rclpy's Node
class Talker(Node):
    def __init__(self):
        # Initialize the parent class (Node) with the name "data_turtle"
        super().__init__("data_turtle")

        # Create a publisher that publishes String messages
        # Topic name = "chatter", Queue size = 10
        self.publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # Create a timer that calls self.timer_callback every 1 second
        self.timer = self.create_timer(1, self.timer_callback)

    # This function runs every time the timer triggers
    def timer_callback(self):
        # Create a new Twist message
        msg = Twist()
        # Fill the message with text + the current counter value
        msg.linear.x = 0.5
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.5
        # Publish the message on the "chatter" topic
        self.publisher.publish(msg)
        # AS PRINT 
        self.get_logger().info(f'Publishing: "{msg}"')  



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
