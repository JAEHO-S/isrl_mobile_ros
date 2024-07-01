# isrl_mobile_robot
This is for isrl 4-differential wheel robot ROS PACKAGE

# Prerequisites

- ubuntu 20.04 + ROS Noetic

- Cuda dependency

- ZED SDK   # If you use ZED camera

# Build workspace

`cd your_workspace/src`

`git clone --recurse-submodules https://github.com/JAEHO-S/isrl_mobile_ros.git`

`cd ..`

`rosdep install --from-paths src --ignore-src -r`

`catkin build`

# Caution

For using IMU sensor you should check the port first. 

`ls /dev/tty*`

If you check the IMU port. Then move to IMU config file.

`cd ~/isrl_ws/src/witmotion_ros/config`

`nano config.yaml`

You can change the port.

`port: ttyUSB0` or `port: ttyUSB1`

NOW using `ttyIMU`, but might be different another pc on robot.

For LiDAR, using `ttyLiDAR` on jetson orin nx.
