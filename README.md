## Requirements:

1. Micro-XRCE-DDS-Agent
2. PX4-Autopilot
3. default.sdf file (in this repo)
4. QGC offboard/takeoff
<!-- <strike> 4. px4_ros_com package: https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard_py/offboard_control.py </strike> -->
5. ros_gz_bridge package: https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge
6. ultralytics_ros package: https://github.com/Alpaca-zip/ultralytics_ros
7. ultralytics_tracker.py file to be replaced in the package. Alongside, a change required in ultralytics_ros>launch>tracker.launch.xml with the param below in pkg=ultralytics_ros <br>
``<param name="classes" value="[0]"/> ``
8. The file refactored_main.py (in this repo)


## Purpose of Each Pkg:

1. default.sdf file to be replaced in folder:  <b> ~/PX4-Autopilot/Tools/simulation/gz/worlds </b>
2. px4_ros_com package: for onboarding <br>
3. ros_gz_bridge package: for publishing gz topics to --> ros2 <br>
6. ultralytics_ros: for publishing detection and tracking results <br>
7. refactored_main.py: main node

## Running:

https://www.notion.so/ROS2-ROS2-1de8042b48ef80d0a244dae87aff3299?source=copy_link
