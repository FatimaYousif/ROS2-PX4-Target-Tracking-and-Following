## Requirements:

1. Micro-XRCE-DDS-Agent
2. PX4-Autopilot
3. default.sdf file (in this repo)
4. px4_ros_com package: https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard_py/offboard_control.py
5. ros_gz_bridge package: https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge
6. PX4-ROS2-Drone-Composable package: https://github.com/Fixit-Davide/PX4-ROS2-Drone-Composable/tree/master
7. tf2_ros
8. ultralytics_ros package: https://github.com/Alpaca-zip/ultralytics_ros
9. The file todo_tf.py (in this repo)


## Purpose of Each Pkg:

1. default.sdf file to be replaced in folder:  <b> ~/PX4-Autopilot/Tools/simulation/gz/worlds </b>
2. px4_ros_com package: for onboarding <br>
3. ros_gz_bridge package: for publishing gz topics to --> ros2 <br>
4. PX4-ROS2-Drone-Composable: to publish dynamic transforms from map --> baselink frame <br>
5. tf2_ros: for publishing static transform from baselink --> camera frame <br>
6. ultralytics_ros: for publishing detection and tracking results <br>
7. todo_tf.py: main node

## Running:

https://www.notion.so/ROS2-ROS2-1de8042b48ef80d0a244dae87aff3299?source=copy_link
