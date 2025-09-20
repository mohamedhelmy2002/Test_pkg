import rclpy
from rclpy.node import Node
# Import the standard ROS 2 message type "String"
from std_msgs.msg import String
from geometry_msgs.msg import Twist
# for data from joystick
import ast


# Define a class Pubsub that inherits from rclpy's Node
class Pubsub(Node):
    def __init__(self):
        # Initialize the parent class (Node) with the name "pubsub"
        super().__init__("pubsub")
        # Create a publisher that publishes String messages
        self.publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        freq = 10  # Hz
        period = 1.0 / freq  # seconds
        # Create a timer that calls self.timer_callback every 1.0 seconds
        self.timer = self.create_timer(period, self.timer_callback)
        
        # create a subscriber that listens on the "chatter" topic
        self.subscription = self.create_subscription(String,"/gamepad",self.callback,10)

        #data from joystick
        self.x=0.0
        self.z=0.0
        self.move=0
        self.pre_move=0    # previous move value
        self.turn=0
        self.pre_turn=0 # previous turn value
    # Callback function that runs every time a new message is received
    def callback(self, msg):
        # Print the message contents
        self.get_logger().info(f"I heard: {msg.data}")
        data=msg.data
        # convert string representation of list to actual list
        buttons_str = data.split("Buttons:")[1].split("Axes:")[0].strip()
        axes_str = data.split("Axes:")[1].strip()

        #convert string representation of list to actual list
        buttons=ast.literal_eval(buttons_str)
        axes=ast.literal_eval(axes_str)
        if axes[1]!=0 :
            axes[1]=-1*axes[1]  # to correct the direction

            if self.move!=0 and (self.move*self.pre_move)<0 :
                self.x=0
            else:
                self.move=axes[1]*0.2
                self.x=self.x+self.move

            self.pre_move=self.move
        elif axes[1]==0 :
            self.x=0
            self.move=0
        if axes[3]!=0 : 
            axes[3]=-1*axes[3]  # to correct the direction

            if self.turn!=0 and (self.turn*self.pre_turn)<0 :
                self.z=0
            else:
                self.turn=axes[3]*0.2
                self.z=self.z+self.turn
                
            self.pre_turn=self.turn
        elif axes[3]==0 :
            self.turn=0
            self.z=0
    # This function runs every time the timer triggers
    def timer_callback(self):
        # Create a new Twist message
        msg = Twist()
        # Fill the message with text + the current counter value
        msg.linear.x = float(self.x)if self.x is not None else 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(self.z)if self.z is not None else 0.0

        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg}"')

def main():
    rclpy.init()
    node = Pubsub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


# Standard Python entry point check
# Ensures main() only runs if this file is executed directly
if __name__ == "__main__":
    main()
