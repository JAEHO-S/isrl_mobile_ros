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

<img width="407" height="406" alt="그림2" src="https://github.com/user-attachments/assets/39ed4398-13c3-4a1e-82a8-6db15f8cc1ee" />

<img width="407" height="407" alt="그림1" src="https://github.com/user-attachments/assets/080e1893-d015-4e39-8dc7-291d52c1eeed" />

