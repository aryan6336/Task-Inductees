import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen, Spawn, TeleportAbsolute
import math
import time

class TurtleDraw(Node):
    def __init__(self):
        super().__init__('turtle_draw')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')

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

    def teleport(self, x, y, theta=0.0):
        """Moves the turtle instantly to a specific position."""
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teleport service...')
        req = TeleportAbsolute.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        self.teleport_client.call_async(req)
        time.sleep(0.5)

    def draw_line(self, x, y):
        """Moves to (x, y) drawing a line."""
        self.teleport(x, y)

    def draw_circle(self, x, y, radius=1.0):
        """Moves to center of the circle and draws it."""
        self.teleport(x, y - radius)  # Start at bottom of circle
        self.set_pen(off=False)
        circumference = 2 * math.pi * radius
        speed = 1.0
        angular_speed = speed / radius
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = angular_speed
        self.publisher_.publish(msg)
        time.sleep(circumference / speed)  # Wait until circle is drawn
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def draw_shape(self):
        """Draws the full drone-like shape as per the image."""
        self.set_pen(255, 255, 255, 2, off=False)

        # Draw the square
        points = [(5, 7), (7, 5), (5, 3), (3, 5), (5, 7)]
        for x, y in points:
            self.draw_line(x, y)

        # Draw connecting lines to circles
        connectors = [(2, 8), (8, 8), (8, 2), (2, 2)]
        for x, y in connectors:
            self.draw_line(x, y)

        # Draw external circles
        circles = [(2, 8), (8, 8), (8, 2), (2, 2)]
        for x, y in circles:
            self.draw_circle(x, y, 1)

def main(args=None):
    rclpy.init(args=args)
    turtle_draw = TurtleDraw()
    turtle_draw.draw_shape()
    turtle_draw.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
