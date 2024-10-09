import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
import sys
import tty
import termios
import threading

class CustomTeleop(Node):
    def __init__(self):
        super().__init__('custom_teleop_node')
        
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pen_service = self.create_client(SetPen, '/turtle1/set_pen')
        self.speed = 1.0  
        self.current_color = (0, 0, 0) 
        self.pen_service.wait_for_service()
        
        self.stop_key = False
        
        threading.Thread(target=self.key_listener, daemon=True).start()
        
        self.move_turtle()

    def key_listener(self):
        """Listen for key presses to control the turtle."""
        
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        try:
            while not self.stop_key:
                key = sys.stdin.read(1)
                self.process_key(key)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def process_key(self, key):
        """Process the pressed key and control the turtle."""
        if key == 'w':  # Move forward
            self.move_turtle(linear=self.speed)
        elif key == 's':  # Move backward
            self.move_turtle(linear=-self.speed)
        elif key == 'a':  # Turn left
            self.move_turtle(angular=self.speed)
        elif key == 'd':  # Turn right
            self.move_turtle(angular=-self.speed)
        elif key == 'q':  # Stop
            self.move_turtle(linear=0.0, angular=0.0)
        elif key == '1':  # Set speed to 0.5
            self.speed = 0.5
            self.get_logger().info("Speed set to 0.5")
        elif key == '2':  # Set speed to 1.0
            self.speed = 1.0
            self.get_logger().info("Speed set to 1.0")
        elif key == '3':  # Set speed to 1.5
            self.speed = 1.5
            self.get_logger().info("Speed set to 1.5")
        elif key == 'r':  # Change pen color to red
            self.set_pen_color(255, 0, 0)
        elif key == 'g':  # Change pen color to green
            self.set_pen_color(0, 255, 0)
        elif key == 'b':  # Change pen color to blue
            self.set_pen_color(0, 0, 255)
        elif key == '\x03':  # Exit on Ctrl+C
            self.stop_key = True
            self.move_turtle(0.0, 0.0)  # Stop the turtle

    def move_turtle(self, linear=0.0, angular=0.0):
        """Publish movement commands to the turtle."""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)

    def set_pen_color(self, r, g, b):
        """Set the pen color."""
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = 2
        req.off = 0
        self.pen_service.call_async(req)
        self.current_color = (r, g, b)
        self.get_logger().info(f"Pen color set to RGB: {self.current_color}")

def main(args=None):
    rclpy.init(args=args)
    teleop_node = CustomTeleop()
    try:
        rclpy.spin(teleop_node)
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
