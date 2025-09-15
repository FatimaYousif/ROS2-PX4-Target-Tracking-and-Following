## Requirements:

1. Micro-XRCE-DDS-Agent
2. PX4-Autopilot
3. px4_ros_com package: https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard_py/offboard_control.py
4. ros_gz_bridge package: https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge
5. PX4-ROS2-Drone-Composable package: https://github.com/Fixit-Davide/PX4-ROS2-Drone-Composable/tree/master
6. tf2_ros
7. ultralytics_ros package: https://github.com/diogoferreira08/ultralytics_ros/tree/humble-devel
8. The file todo_tf.py


## Purpose of Each Pkg:

1. px4_ros_com package: for onboarding <br>
2. ros_gz_bridge package: for publishing gz topics to --> ros2 <br>
3. PX4-ROS2-Drone-Composable: to publish dynamic transforms from map --> baselink frame <br>
4. tf2_ros: for publishing static transform from baselink --> camera frame <br>
5. ultralytics_ros: for publishing detection and tracking results <br>
6. todo_tf.py: main node

## Running:

https://www.notion.so/ROS2-ROS2-1de8042b48ef80d0a244dae87aff3299?source=copy_link
