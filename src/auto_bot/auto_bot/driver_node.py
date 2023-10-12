import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class VelPublisher(Node):

    def __init__(self):
        super().__init__('vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x+=0.5
        msg.angular.z+=0.3
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    vel_publisher = VelPublisher()

    rclpy.spin(vel_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vel_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()