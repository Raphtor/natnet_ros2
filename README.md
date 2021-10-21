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

This should auto-discover a server, but if you want, you can pass in a server as a ros parameter.

The node will publish to `<tracker_name>/pose_stamped`, where `<tracker_name>` is the name of the rigid body defined in Motive. The default coordinate system is the one used by Motive.

### Options and ROS parameters:

You can set these parameters at runtime using the ROS parameter service, e.g
```
ros2 run natnet client use_timestamps:=false
```


* `server_address` 

  The address to look for the server. If left unset (default), the node will auto-discover a server.

* `use_timestamps` 

  If `true`, publish `PoseStamped` messages to `<tracker_name>/pose_stamped`. Otherwise, publish to `Pose` messages to `<tracker_name>/pose`.

* `world_frame_id`

   The frame from which to publish the messages to. Default: `world`.

* `discovery_timeout`

   How long to wait discovering servers before timing out, in float seconds. Default: `30.0`.

* `local_address`

  The local address of the computer running this node. If unset (default), this will use the address returned by the auto-discovery.
