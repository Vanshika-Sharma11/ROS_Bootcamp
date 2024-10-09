import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from std_srvs.srv import Empty
import time
import math

class DrawCirclesNode(Node):
    def __init__(self):
        super().__init__('draw_circles_node')
        
        
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.pen_service = self.create_client(SetPen, '/turtle1/set_pen')
        self.reset_service = self.create_client(Empty, '/reset')
        
        
        self.pen_service.wait_for_service()
        self.reset_service.wait_for_service()
        
       
        self.reset_simulation()

        self.circles = [
            {'color': (255, 0, 0), 'radius': 1.0},  
            {'color': (0, 255, 0), 'radius': 1.5},  
            {'color': (0, 0, 255), 'radius': 2.0} 
        ]

        self.current_circle_index = 0
        self.start_position = (5.5, 5.5)  

        
        self.draw_next_circle()

    def reset_simulation(self):
        """Reset the turtlesim simulation."""
        reset_request = Empty.Request()
        self.reset_service.call_async(reset_request)

    def set_pen(self, r, g, b, width=2, off=0):
        """Set the pen color and width."""
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        self.pen_service.call_async(req)

    def draw_next_circle(self):
        """Draw the next circle if available."""
        if self.current_circle_index < len(self.circles):
            circle = self.circles[self.current_circle_index]
            self.set_pen(circle['color'][0], circle['color'][1], circle['color'][2])  
            self.draw_circle(circle['radius'])  

    def draw_circle(self, radius):
        """Draw a circle with the specified radius."""
        
        angular_speed = 1.0  
        linear_speed = angular_speed * radius  

        total_time = 2 * math.pi * radius / linear_speed 
        steps = int(total_time / 0.1) 
        for _ in range(steps):
            msg = Twist()
            msg.linear.x = linear_speed
            msg.angular.z = angular_speed
            self.publisher_.publish(msg)
            time.sleep(0.1)  

        self.stop_movement()
        
        self.current_circle_index += 1
        time.sleep(1)  
        self.draw_next_circle()  

    
    def stop_movement(self):
        """Stop the turtle's movement."""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    draw_circles_node = DrawCirclesNode()
    rclpy.spin(draw_circles_node)
    draw_circles_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
