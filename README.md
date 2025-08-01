Husky Master
=============

The base image used on the Clearpath Husky A200. It uses Ubuntu 24.04 and ROS2 Jazzy, and installs all needed dependencies and scripts to customize, simulate, and bringup the hardware platform.

### How to Build and Use?
```
git clone https://github.com/b3prime/husky_master_jazzy.git
cd dcist_master_ros2 && git submodule update --init --recursive
sudo ./build.bash
sudo ./run.bash
```

### How to run the Husky
```
ros2 launch /etc/clearpath/platform/launch/platform-service.launch.py
```
To edit configuration files related to the Husky's bringup, navigate to: /etc/clearpath/platform/config
