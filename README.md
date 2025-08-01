The base image used on the Clearpath Husky A200. It uses Ubuntu 24.04 and ROS2 Jazzy, and installs all needed dependencies and scripts to customize, simulate, and bringup the hardware platform.

**How to Build and Use?**

'''
git clone https://github.com/KumarRobotics/dcist_master_ros2.git
cd dcist_master_ros2 && git submodule update --init --recursive
./build.bash dcist-master-jazzy x86_64_nvda
./run.bash dcist-master-jazzy-nvda:latest


