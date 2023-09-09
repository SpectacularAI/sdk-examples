# ROS wrapper

## Supported platforms

* ROS2 Humble on Linux

## Dependencies

ROS2 Hubmle framework:

* https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Binary.html

## Build and Run

Make sure to have your ROS environment sourced i.e. `ros2` on command line works and that you have OAK-D device connected

From this folder, run:
```
colcon build
source install/setup.bash
ros2 launch launch/mapping.py
```
