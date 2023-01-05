#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash;
echo 'ros2 imu tools'
git clone https://gitlab.com/boldhearts/ros2_imu_tools.git
rosdep install --from-paths . --ignore-src -r -y;
