import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen, Spawn, Kill
import math
import time

class TurtleDraw(Node):
    def __init__(self):
        super().__init__('turtle_draw')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, 'kill')

    def set_pen(self, r=255, g=255, b=255, width=2, off=False):
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for pen service...')
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        self.pen_client.call_async(req)

    def draw_line(self, distance, speed=1.0):
         msg = Twist()
         msg.linear.x = speed
         self.publisher_.publish(msg)

         start_time = self.get_clock().now()  # Get the start time
         duration = distance / speed  # Time required to travel the distance

         while (self.get_clock().now() - start_time).nanoseconds < int(duration * 1e9):
             self.publisher_.publish(msg)
             rclpy.spin_once(self, timeout_sec=0.1)  # Allows ROS2 to handle callbacks

    
         msg.linear.x = 0.0
         self.publisher_.publish(msg)
         time.sleep(0.5)



    def draw_square(self, side_length):
        for _ in range(4):
            self.draw_line(side_length)
            self.turn(90)

    def draw_circle(self, radius):
        circumference = 2 * math.pi * radius
        speed = 1.0
        angular_speed = speed / radius
        duration = circumference / speed  # Add this line to define duration

        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = angular_speed
        self.publisher_.publish(msg)
        self.get_logger().info(f'Drawing circle with radius {radius}')

        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds < int(duration * 1e9):  # Now 'duration' is defined
            self.publisher_.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        time.sleep(0.5)



    def turn(self, angle, speed=30):
        msg = Twist()
        msg.angular.z = math.radians(speed if angle > 0 else -speed)  # Set rotation direction
    
        start_time = self.get_clock().now()
        target_time = abs(angle) / speed  # Time needed to complete the turn

        while (self.get_clock().now() - start_time).nanoseconds < int(target_time * 1e9):
           self.publisher_.publish(msg)
           rclpy.spin_once(self, timeout_sec=0.05)  # Small time step for smooth turning

        # Stop rotation smoothly
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        time.sleep(0.2)  # Allow stabilization

 
    def pen_up(self):
        self.set_pen(off=True)

    def pen_down(self):
        self.set_pen(off=False)

    def draw_drone(self):
        self.turn(270)
        self.pen_down()
        self.draw_line(2.82)
        self.pen_up()
        self.turn(180)
        self.draw_line(1)
        self.turn(90)
        self.pen_down()
        self.draw_circle(1)
        self.pen_up()
        self.turn(270)
        self.draw_line(1.82)
        self.turn(270)
        

def main(args=None):
    rclpy.init(args=args)
    turtle_draw = TurtleDraw()
    turtle_draw.pen_up()
    turtle_draw.draw_line(2)
    turtle_draw.pen_down()
    turtle_draw.turn(135)
    turtle_draw.draw_square(2.82)
    turtle_draw.pen_up()
    turtle_draw.draw_line(1.41)
    turtle_draw.draw_drone()
    for _ in range(3):
        turtle_draw.draw_line(1.41)
        turtle_draw.turn(90)
        turtle_draw.draw_line(1.41)
        turtle_draw.draw_drone()

    turtle_draw.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
