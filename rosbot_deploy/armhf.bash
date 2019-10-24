#!/bin/bash
. /opt/ros/dashing/setup.bash
cd /ws/robot_ws
rosdep install --from-paths src  --rosdistro dashing --ignore-src -r -y
colcon build --build-base armhf_build --install-base armhf_install
colcon bundle --build-base armhf_build --install-base armhf_install --bundle-base armhf_bundle --apt-sources-list /opt/cross/apt-sources.yaml
exit