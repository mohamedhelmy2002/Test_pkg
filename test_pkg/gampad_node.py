import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame

# Define a class Talker that inherits from rclpy's Node
class Talker(Node):
    def __init__(self):
        # joystick init
        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.num_buttons = self.joystick.get_numbuttons()
        self.num_axes = self.joystick.get_numaxes()

        # node init
        super().__init__("talker")
        self.publisher = self.create_publisher(String, "/gamepad", 10)

        frq=10  # Hz
        period=1.0/frq  # seconds
        # Create a timer that calls self.timer_callback every 1.0 seconds
        self.timer = self.create_timer(period, self.timer_callback)

    # This function runs every time the timer triggers
    def timer_callback(self):
        pygame.event.pump()  # updates internal joystick state
        # get button states
        buttons = [self.joystick.get_button(i) for i in range(self.num_buttons)]
        # get axis values (-1.0 to +1.0)
        axes = [round(self.joystick.get_axis(i), 2) for i in range(self.num_axes)]
        # Create a new String message
        msg = String()
        # Fill the message with text + the current counter value
        msg.data = f"Buttons:{buttons} Axes:{axes}"
        # Publish the message on the "chatter" topic
        self.publisher.publish(msg)
        # AS PRINT
        self.get_logger().info(msg.data)



def main():
    rclpy.init()
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
