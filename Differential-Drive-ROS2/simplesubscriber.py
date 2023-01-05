import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from motor_driver import *


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.motor_driver = MotorDriver( i_BASE_PWM=50,
                                         i_MULTIPLIER_STANDARD=0.1,
                                         i_MULTIPLIER_PIVOT=1,
                                         simple_mode=True)

        self.subscription = self.create_subscription(Twist,
                                                    'cmd_vel',
                                                    self.cmd_vel_callback,
                                                    10)
        self.subscription  # prevent unused variable warning
    
    def cmd_vel_callback(self, msg):
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        # Decide Speed
        self.motor_driver.set_cmd_vel(linear_speed, angular_speed)
        self.get_logger().info('I heard linear Speed: "%s"' %linear_speed)
        self.get_logger().info('I heard angular Speed: "%s"' %angular_speed)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
