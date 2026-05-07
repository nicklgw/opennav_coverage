#!/bin/bash

export ROS_DOMAIN_ID=5
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

mkdir -m a=rwx -p /var/log/bzlrobot/opennav_coverage_task
export ROS_LOG_DIR="/var/log/bzlrobot/opennav_coverage_task"

/usr/bin/python3 /opt/ros/humble/bin/ros2 launch opennav_coverage_task coverage_task.launch.py # 启动脚本
