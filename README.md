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

#### NOTES
[STILL DEBUGGING]
- Should follow the structure: ``ros2 launch <package_name> <launch_file_name>``
- Need to add an `exec_depend` dependency inside `package.xml`


## Current tree:

```
C:.
│   CMakeLists.txt
│   LICENSE
│   package.xml
│   README.md
│
├───build
│   │   .built_by
│   │   COLCON_IGNORE
│   │
│   └───localisation_aruco_marker
│           cmake_args.last
│           colcon_build.rc
│           colcon_command_prefix_build.bat
│           colcon_command_prefix_build.bat.env
│
├───include
│   └───localisation_aruco_marker
├───install
│       .colcon_install_layout
│       COLCON_IGNORE
│       local_setup.bat
│       local_setup.ps1
│       setup.bat
│       setup.ps1
│       _local_setup_util_bat.py
│       _local_setup_util_ps1.py
│
├───launch
│   │   =0.9.11
│   │   localisation_aruco_marker.py
│   │
│   └───__pycache__
│           localisation_aruco_marker.cpython-312.pyc
│
├───log
│   │   COLCON_IGNORE
│   │
│   └───build_2025-02-19_12-42-31
│       │   events.log
│       │   logger_all.log
│       │
│       └───localisation_aruco_marker
│               command.log
│               stderr.log
│               stdout.log
│               stdout_stderr.log
│               streams.log
│
├───scripts
│       localisation_aruco_marker.py
│
└───src
        localisation_aruco_marker.cpp
```
