#!/usr/bin/env python

"""
imu_node.py

"""
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Imu, Temperature
from geometry_msgs.msg import Vector3
import constants
from mpu6050 import mpu6050

class IMUNode(Node):

    def __init__(self):
        """Create a IMUNode.
        """
        super().__init__("imu_node")
        self.get_logger().info("imu_node started.")
        self.stop_queue = threading.Event()
        ## following params can now be configured in config file config/imu_params.yaml
        self.imu_frame = self.declare_parameter("~imu_frame", constants.IMU_FRAME).value
        self.pub_topic = self.declare_parameter("~imu_pub_topic", constants.IMU_MSG_TOPIC).value
        self.publish_rate = self.declare_parameter("~publish_rate", constants.IMU_MSG_RATE).value
        self.get_logger().info(f"imu_frame:: {self.imu_frame} pub_topic:: {self.pub_topic} publish_rate::{self.publish_rate}")

        # Publisher that sends combined sensor messages with IMU acceleration and gyroscope data.
        self.imu_message_pub_cb_grp = ReentrantCallbackGroup()
        self.imu_message_publisher = self.create_publisher(Imu,
                                                            self.pub_topic,
                                                            1,
                                                            callback_group=self.imu_message_pub_cb_grp)

        # Heartbeat timer.
        self.timer_count = 0
        self.timer = self.create_timer(5.0, self.timer_callback)

        self.get_logger().info(f"IMU node successfully created. publishing on topic:: {self.pub_topic}")


    def timer_callback(self):
        """Heartbeat function to keep the node alive.
        """
        self.get_logger().debug(f"Timer heartbeat {self.timer_count}")
        self.timer_count += 1

    def __enter__(self):
        """Called when the node object is created using the 'with' statement.
        Returns:
           IMUNode : self object returned.
        """
        try:
            self.sensor = mpu6050(constants.MPU6050_ADDR)
            # if permission denied for i2C follow :https://ask.wingware.com/question/3/i2c-problem-with-remote-raspberry-pi/
            # Temporary Solution : sudo chmod a+rw /dev/i2c-*
            # Permanent Solution : Edit the file /etc/udev/rules.d/99-com.rules
                                # If this line exists:
                                # SUBSYSTEM=="ic2-dev", GROUP="i2c", MODE="0660"
                                # then change the MODE to "0666".
                                # If it does not exist then add it with the MODE="0666" and also note the "==".
                                # Reboot.

            # Defining the Range for Accelerometer and Gyroscope
            self.sensor.set_accel_range(mpu6050.ACCEL_RANGE_4G)
            self.sensor.set_gyro_range(mpu6050.GYRO_RANGE_250DEG)

        except Exception as ex:
            self.get_logger().info(f"Failed to create IMU monitor: {ex}")
            self.observer = None
            raise ex

        self.get_logger().info('Initialization and calibration of IMU sensor done.')

        self.thread = threading.Thread(target=self.processor)
        self.thread.start()

        # Start IMU event monitor.
        return self

    def __exit__(self, ExcType, ExcValue, Traceback):
        """Called when the object is destroyed.
        """
        self.get_logger().info('Exiting.')
        self.stop_queue.set()
        self.rate.destroy()
        self.thread.join()
        

    def processor(self):

        self.get_logger().info(f"Publishing messages at {self.publish_rate} Hz.")
        self.rate = self.create_rate(self.publish_rate)

        while not self.stop_queue.is_set() and rclpy.ok():
            try:
                self.publish_imu_message()
                self.rate.sleep()
            except Exception as ex:
                self.get_logger().error(f"Failed to create IMU message: {ex}")      
        
    def publish_imu_message(self):
        """Publish the sensor message when we get new data for the slowest sensor(LiDAR).
        """
        try:
            imu_msg = Imu()
            temp_msg = Temperature()
            data = self.sensor.get_all_data()

            # fetch all gyro values - return in rad / sec
            # update for fork - original repo had x & y swapped and z values reversed/upside down. Due to this the IMU data was not showing correct
            # so fixed the swap and z direction for both gyro and accel values

            accel = Vector3()
            accel.x = data[0]['x']
            accel.y = data[0]['y']
            accel.z = data[0]['z']

            gyro = Vector3()
            gyro.x = data[1]['x']
            gyro.y = data[1]['y']
            gyro.z = data[1]['z']
            
            imu_msg.angular_velocity = gyro
            imu_msg.angular_velocity_covariance = constants.EMPTY_ARRAY_9
            imu_msg.linear_acceleration = accel
            imu_msg.linear_acceleration_covariance = constants.EMPTY_ARRAY_9
            imu_msg.orientation_covariance = constants.EMPTY_ARRAY_9
            imu_msg.orientation_covariance[0] = -1.0
            
            # add header
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.imu_frame


            self.get_logger().debug('gz: {:+.0f}'.format(gyro.z))

            self.imu_message_publisher.publish(imu_msg)

        except Exception as ex:
            self.get_logger().error(f"Error in publishing sensor message: {ex}")


def main(args=None):
    
    try:
        rclpy.init(args=args)
        with IMUNode() as imu_node:
            executor = MultiThreadedExecutor()
            rclpy.spin(imu_node, executor)
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        imu_node.destroy_node()
    except KeyboardInterrupt:
        pass    
    rclpy.shutdown()


if __name__ == "__main__":
    main()
