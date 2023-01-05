# Differential-Drive-ROS2

This contain differential drive program for N20 Motors with Raspberry Pi

## For i2C Detecting MPU6050

`sudo i2cdetect -r -y 1`

![alt](https://)image.png

Let's create a directory for your project, initialize a ROS 2 workspace and give your robot a name. In our case **mob_diff_bot**

    ros2 pkg create --build-type ament_cmake mob_diff_bot

Since we need to add more files to, lets make robot *description* folder, *launch* folder and *config* folder.

    mkdir launch description config

We will be using *sam_bot_description*, from [Nav2](https://navigation.ros.org/setup_guides/urdf/setup_urdf.html) as reference, but splitting the urdf for Gazebo and pheripheral components. We are using following compoinets for the mobile robot.

    1. Raspberry Pi 4 8GB
    2. ydLidar
    3. MPU6050 - Imu
    4. Cytron Makerdrive
    5. N20 Motors
    6. Raspberry Pi Camera
    7. Oak-D S2 Camera

We are now ready to build our project using colcon. Navigate to the project root and execute the following commands.

    colcon build
    . install/setup.bash

To kill all Gazebo Server (if necessary)
    killall -9 gzserver

To run the URDF yo need to execute 
    1. sudo apt install ros-foxy-xacro
    2. sudo apt install ros-foxy-launch-ros

[^1]: Created by Nirmal Thomas, for learing and Understanding of Ros 2 Navigation. 