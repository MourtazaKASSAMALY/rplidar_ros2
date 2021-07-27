Original repository: https://github.com/youngday/rplidar_ros2 | See rplidar_ros2.patch

# What's new with this fork

Launch file with configurable parameters:

- serial_port, defaults to tty/USB0
- serial_baudrate, defaults to 256000
- frame_id, defaults to base_scan
- inverted, defaults to false
- angle_compensate, defaults to true
- scan_mode, defaults to Sensitivity
- topic_name, defaults to /scan

RPLIDAR ROS package
=====================================================================

ROS node and test application for RPLIDAR

Visit following Website for more details about RPLIDAR:

rplidar roswiki: http://wiki.ros.org/rplidar

rplidar HomePage:   http://www.slamtec.com/en/Lidar

rplidar SDK: https://github.com/Slamtec/rplidar_sdk

rplidar Tutorial:  https://github.com/robopeak/rplidar_ros/wiki

How to build rplidar ros package
=====================================================================

## Requirements:

- Ubuntu 18.04.5 LTS
- ROS2 Eloquent

## Create ROS2 Workspace:

```shell
cd ~
mkdir -p ros2_worskpace/src
echo "source ~/ros2_workspace/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Build custom RP LiDAR ROS2 packages:

```shell
cd ~/ros2_workspace/src
git clone -b ros2 https://github.com/MourtazaKASSAMALY/rplidar_ros2.git
cd ..
colcon build --symlink-install
source ~/.bashrc
```

How to run rplidar ros2 package
=====================================================================
There're three ways to run rplidar ros package

I. Run rplidar node and view in the rviz
------------------------------------------------------------
At first console,
```bash
source ./install/setup.bash
ros2 run rplidar_ros2 rplidarNode 
```
At second console,
```bash
source ./install/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0  world laser_frame  
```
At third console,
```bash
source ./install/setup.bash
rviz2 ./install/rplidar_ros2/share/rplidar_ros2/rviz/rplidar.rviz
```
Rviz is set, set frame into laser_frame instead not laser, add a laser scan with /scan topic and you should see rplidar's scan result in the rviz.

II. Run rplidar node and view using test application
------------------------------------------------------------
```bash
source ./install/setup.bash
ros2 run rplidar_ros2 rplidarNodeClient 
```
You should see rplidar's scan result in the console

Notice: the different is serial_baudrate between A1/A2 and A3/S1

III. Run rplidar A3 launch file with configurable parameters (preferred)
------------------------------------------------------------
```bash
ros2 launch rplidar_ros2 rplidar_a3.launch.py topic_name:=/scan
```

RPLidar frame
=====================================================================
RPLidar frame must be broadcasted according to picture shown in rplidar-frame.png
