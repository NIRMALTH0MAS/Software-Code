from pickle import TRUE
import rclpy
from rclpy.node import Node
import sys
from geometry_msgs.msg import Twist
from motor_driver import MotorDriver

class RobotMover(object):

    def __init__(self, value_BASE_PWM, value_MULTIPLIER_STANDARD, value_MULTIPLIER_PIVOT, value_simple_mode):
        # rclpy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        self.motor_driver = MotorDriver( i_BASE_PWM=value_BASE_PWM,
                                         i_MULTIPLIER_STANDARD=value_MULTIPLIER_STANDARD,
                                         i_MULTIPLIER_PIVOT=value_MULTIPLIER_PIVOT,
                                         simple_mode=value_simple_mode)

        self.subscription = self.create_subscription(Twist,'cmd_vel',self.cmd_vel_callback,10)
        self.get_logger().info('ACR Started')
        # rospy.loginfo("ACR Started...")

    def cmd_vel_callback(self, msg):
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        # Decide Speed
        self.motor_driver.set_cmd_vel(linear_speed, angular_speed)


    def listener(self):
        # rospy.spin()
        rclpy.spin()

if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('acr_diff_cmd_vel_listener')
    # rospy.init_node('acr_diff_cmd_vel_listener', anonymous=True)

    if len(sys.argv) > 5:
        value_BASE_PWM = int(float(sys.argv[1]))
        value_MULTIPLIER_STANDARD = float(sys.argv[2])
        value_MULTIPLIER_PIVOT = float(sys.argv[3])
        value_simple_mode = sys.argv[4] == "true"

        robot_mover = RobotMover(value_BASE_PWM,
                                 value_MULTIPLIER_STANDARD,
                                 value_MULTIPLIER_PIVOT,
                                 value_simple_mode)
        robot_mover.listener(Node)
    # robot_mover = RobotMover(50,
    #                         0.1,
    #                         1,
    #                         True)
    # robot_mover.listener(Node)
