# ROS2-Localisation-System 

This is a copy of the ROS2 localisation system scripts handed over by Clayder - following on from Alex's work in ROS1. Scripts need to be edited in order to function with the ROS2 environment.

## Locations on computers
LOCAL: C:\Users\nhkje\git\ROS2-Localisation-System

Raspi: ~/ros2_ws/src/ROS2-Localisation-System

## Building Environment
Using `colcon build` tools:
1. `cd ~/ros2_ws/`
2. Source ROS2: `source /opt/ros/rolling/setup.bash`
3. Source workspace setup script: `source install/setup.bash`
4. Build package: `colcon build --packages-select localisation_aruco_marker`
5. Source workspace script again: `source install/setup.bash`
6. Run test: `colcon test --packages-select localisation_aruco_marker`


<!-- !! WAS ABLE TO RUN WITHOUT ANY LAUNCH DEPENDENCIES !! -->
<!-- ## Setup `package.xml` -->
<!-- NOTE - 'package.xml' must be checked whenever converting between ROS2 versions, as the launch dependency -->
<!-- names change due to updates over time. -->
<!--  -->
<!-- ### `package.xml` in ROLLING -->
<!-- ``` -->
  <!-- <exec_depend>launch</exec_depend> -->
  <!-- <exec_depend>launch_ros</exec_depend> -->
<!-- ``` -->
<!--  -->
<!-- ### `package.xml` in FOXY -->
<!-- ``` -->
<!-- <exec_depend>ros2launch</exec_depend> -->
<!-- ``` -->


## Running nodes
### localisation_aruco_marker.cpp
1. `cd ~/ros2_ws/`
2. Source ROS2: `source /opt/ros/rolling/setup.bash`
3. Source workspace setup script: `source install/setup.bash`
4. Run .cpp script: `ros2 run localisation_aruco_marker localisation_aruco_marker`

### localisation_aruco_marker.py
This runs the MarkingTrackerNode() script, to determine the position of the aruco marker.
1. `cd ~/ros2_ws/`
2. Source ROS2: `source /opt/ros/rolling/setup.bash`
3. Source workspace setup script: `source install/setup.bash`
4. Run the node: `ros2 run localisation_aruco_marker localisation_aruco_marker.py`

### localisation_aruco_marker_launch.py
This launches the static transformation nodes, defining the global positions and transforms required to localise the rover centrepoint.
1. `cd ~/ros2_ws/`
2. Source ROS2: `source /opt/ros/rolling/setup.bash`
3. Source workspace setup script: `source install/setup.bash`
4. Go to launch file: `cd ~/ros2_ws/src/ROS2-Localisation-System/launch`
5. Launch node file: `ros2 launch localisation_aruco_marker_launch.py`

## Testing the script
The current plan is to firstly recreate the first experiment within 
the ROS2 environment. Then, work can be done to improving the system.

### Running the rosbag
1. `cd ~/ros2_ws/`
2. Source ROS2: `source /opt/ros/rolling/setup.bash`
3. Source workspace setup script: `source install/setup.bash`
4. `cd ~/ros2_ws/bag_files/ros2_bag_dir`
5. Run rosbag: `ros2 bag play <bagname>`


#### NOTES
[STILL DEBUGGING]
- Should follow the structure: ``ros2 launch <package_name> <launch_file_name>``
- Need to add an `exec_depend` dependency inside `package.xml`

