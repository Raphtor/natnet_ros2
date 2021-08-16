# NatNet package for ROS2


This is a quick and dirty integration of Naturalpoint's NatNet SDK for Optitrack and ROS2.

## Installation

```
mkdir -p ~/natnet_ws/src
cd ~/natnet_ws/src
git clone https://github.com/Raphtor/natnet_ros2.git
cd ..
colcon build --packages-select natnet

```

## Running

```
ros2 run natnet client
```

This should auto-discover a server, but if you want, you can pass in a server as a ros parameter

```
ros2 run natnet client server_address:=192.168.0.100
```