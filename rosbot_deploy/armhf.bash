#!/bin/bash
echo "Set content for /opt/cross/apt-sources.yaml"
echo '# ARM Support
deb [arch=armhf] http://ports.ubuntu.com/ubuntu-ports/ bionic main restricted universe multiverse
deb [arch=armhf] http://ports.ubuntu.com/ubuntu-ports/ bionic-updates main restricted universe multiverse
deb [arch=armhf] http://ports.ubuntu.com/ubuntu-ports/ bionic-backports main restricted
deb [arch=armhf] http://ports.ubuntu.com/ubuntu-ports/ bionic-security main restricted universe multiverse

# Source Repositories
deb-src http://us.archive.ubuntu.com/ubuntu/ bionic main restricted universe multiverse
deb-src http://us.archive.ubuntu.com/ubuntu/ bionic-updates main restricted universe multiverse
deb-src http://us.archive.ubuntu.com/ubuntu/ bionic-backports main restricted universe multiverse
deb-src http://us.archive.ubuntu.com/ubuntu/ bionic-security main restricted universe multiverse

# ROS
deb [arch=armhf] http://packages.ros.org/ros2/ubuntu bionic main' > /opt/cross/apt-sources.yaml
. /opt/ros/dashing/setup.bash
cd /ws/robot_ws
rosdep install --from-paths src  --rosdistro dashing --ignore-src -r -y
colcon build --build-base armhf_build --install-base armhf_install
colcon bundle --build-base armhf_build --install-base armhf_install --bundle-base armhf_bundle --apt-sources-list /opt/cross/apt-sources.yaml
exit