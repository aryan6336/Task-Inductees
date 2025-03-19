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
        time.sleep(distance / speed)
        msg.linear.x = 0.0
        self.publisher_.publish(msg)
        time.sleep(0.5)  # Ensure turtle fully stops

    def draw_square(self, side_length):
        for _ in range(4):
            self.draw_line(side_length)
            self.turn(90)

    def draw_circle(self, radius):
        circumference = 2 * math.pi * radius
        speed = 1.0
        angular_speed = speed / radius
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = angular_speed
        self.publisher_.publish(msg)
        time.sleep(circumference / speed)
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        time.sleep(0.5)

    def turn(self, angle):
        msg = Twist()
        msg.angular.z = math.radians(angle)
        self.publisher_.publish(msg)
        time.sleep(abs(angle) / 90.0)  # Approximate turn timing
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        time.sleep(0.5)

    def pen_up(self):
        self.set_pen(off=True)

    def pen_down(self):
        self.set_pen(off=False)

    def draw_drone(self):
        self.draw_circle(0.5)  # Central body
        positions = [(1, 1), (-1, 1), (-1, -1), (1, -1)]  # Rotor positions
        for x, y in positions:
            self.pen_up()
            self.draw_line(math.sqrt(x**2 + y**2))
            self.pen_down()
            self.draw_circle(0.2)
            self.pen_up()
            self.draw_line(math.sqrt(x**2 + y**2))
        self.pen_down()


def main(args=None):
    rclpy.init(args=args)
    turtle_draw = TurtleDraw()
    turtle_draw.pen_down()
    turtle_draw.draw_square(2.0)
    turtle_draw.pen_up()
    turtle_draw.turn(90)
    turtle_draw.pen_down()
    turtle_draw.draw_circle(1.0)
    turtle_draw.pen_up()
    turtle_draw.draw_line(3.0)
    turtle_draw.pen_down()
    turtle_draw.draw_drone()

    turtle_draw.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
