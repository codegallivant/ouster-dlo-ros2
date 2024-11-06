# ouster-dlo-ros2
Contains ROS2 Humble packages of drivers and direct lidar odometry for an Ouster LIDAR sensor.
  
The packages were cloned from separate GitHub repositories with a few modifications. The packages were edited to use to same System Defaults QoS everywhere. This was required to avoid QoS inequality conflicts.
  
Note: Later on, it may be required to change the QoS used from SystemDefaultsQoS to SensorDataQoS.
  
Ouster drivers were originally cloned from: https://github.com/ouster-lidar/ouster-ros/tree/ros2
  
The DLO package was originally cloned from: https://github.com/Cardinal-Space-Mining/direct_lidar_odometry (Note: The linked repo is a fork of the ROS1 DLO package and it's README has not been updated)

## Prerequisite packages
### For ouster drivers
```bash
ROS_DISTRO=humble
sudo apt install -y             \
    ros-$ROS_DISTRO-pcl-ros     \
    ros-$ROS_DISTRO-tf2-eigen   \
    ros-$ROS_DISTRO-rviz2
sudo apt install -y         \
    build-essential         \
    libeigen3-dev           \
    libjsoncpp-dev          \
    libspdlog-dev           \
    libcurl4-openssl-dev    \
    cmake                   \
    python3-colcon-common-extensions
```
### For DLO
```bash
sudo apt install libomp-dev libpcl-dev libeigen3-dev 
```

## Setup
1. Clone the repo
2. Build
```bash
colcon build
```
3. Source files
```bash
source install/setup.bash
```
4. Obtain the sensor hostname
```bash
avahi-browse -arlt
```
5. Launch ouster drivers
```bash
ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=<sensor host name>
```
6. Launch DLO
```bash
ros2 launch direct_lidar_odometry dlo.launch.py imu_topic:=/ouster/imu pointcloud_topic:=/ouster/points 
```
