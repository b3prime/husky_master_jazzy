Husky Master
=============

The base image used on the Clearpath Husky A200. It uses Ubuntu 24.04 and ROS2 Jazzy, and installs all needed dependencies and scripts to customize, simulate, and bringup the hardware platform.

### SSH'ing into BrainBox
```
ssh -X robot@192.168.8.72
```
**The default password is ```clearpath```**. NOTE: the machine's IPv4 address will change. Run ```ip addr show``` or ```ip a``` to find it. Make sure the -X flag is used (for X11 forwarding).

### How to Build and Use?
```
git clone https://github.com/b3prime/husky_master_jazzy.git
cd husky_master_jazzy && git submodule update --init --recursive
./build.bash
./run.bash
```

### How to Launch The Husky
```
ros2 launch brainbox_husky_bringup brainbox_husky_bringup.launch.py use_sim:={false, true} launch_platform:={false, true}
```
Replace {false, true} with either false or true depending on how you want to launch the platform. use_sim defines whether to launch the Gazebo simulation, and launch_platform defines whether to connect to the A200 over it's serial connection.

To edit configuration files related to the Husky platform, navigate to: /etc/clearpath/platform/config

### SLAM + Path Planning

```
ros2 launch path_planning ouster_slam.launch.py
```

This will launch DLIO and SLAM-toolbox.

If SLAM toolbox is outputting the error:

```
Message Filter dropping message: frame 'os_lidar' at time ______ for reason 'the timestamp on the message is earlier than all the data in the transform cache'
```

Then check the contents of the /tf and /tf_static topics.

### Notes

**Display Errors?**: On the host machine (before building the container), run:
```
sudo xhost +si:localuser:root
```
This will allow any local process running as root to connect to the X11 display server.

Also, ensure that you SSH'd into the machine with the -X flag.

**Can't connect to the Husky over serial?**: BrainBox uses a Serial to USB converter to communicate with the Husky's internal MCU. The converter will appear as something like:
```
/dev/serial/by-id/_
```
The Husky platform package looks for /dev/clearpath/prolific. Ensure that the symlink here is correct.


