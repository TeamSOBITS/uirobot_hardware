#! /bin/bash

echo "╔══╣ Install: Uirobot Hardware (STARTING) ╠══╗"

# Install dependencies
sudo apt update
sudo apt install -y \
    ros-$ROS_DISTRO-rclcpp \
    ros-$ROS_DISTRO-rclcpp-lifecycle \
    ros-$ROS_DISTRO-lifecycle-msgs \
    ros-$ROS_DISTRO-hardware-interface \
    ros-$ROS_DISTRO-pluginlib \
    libserial-dev \

# TODO : Add rules
# sudo udevadm control --reload-rules
# sudo udevadm trigger

# Serial Access
sudo usermod -aG dialout $USERNAME

echo "╚══╣ Install: Uirobot Hardware (FINISHED) ╠══╝"
