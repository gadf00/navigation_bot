# Navigation Bot

This is a project for the Robot Programming course. The `navigation_bot` package provides functionality for a burger robot to navigate through predefined waypoints, add new waypoints, and remove existing ones. The project demonstrates various aspects of robot navigation and control using ROS (Robot Operating System).

## Description

The `navigation_bot` package allows a robot to perform the following operations:

1. **Navigation**: The robot can navigate to predefined waypoints specified in a YAML file.
2. **Add Waypoint**: Users can add new waypoints either by recording the current position of the robot or by manually entering coordinates.
3. **Remove Waypoint**: Users can remove existing waypoints from the list.
4. **Stop Navigation**: Users can stop the robot's navigation during its execution.

## Nodes and Functionality

- **start_position.py**: Initializes the robot's starting position and publishes it to the relevant topics.
- **interface.py**: Provides a user interface for selecting operations such as navigation, adding waypoints, and removing waypoints.
- **choose_waypoint.py**: Allows the user to choose a destination waypoint from a list.
- **navigation.py**: Handles the navigation of the robot to the selected waypoint and provides feedback on the current position.
- **add_waypoint.py**: Allows the user to add new waypoints either by recording the current position or by entering coordinates.
- **remove_waypoint.py**: Allows the user to remove existing waypoints from the list.
- **stop_navigation.py**: Provides a service to stop the robot's navigation during execution.

## Installation and Usage
0. **Prerequisites**:
   ```sh
   cd ~/catkin_ws/src
   git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
   git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
   git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
   cd ~/catkin_ws
   catkin_make
   sudo apt-get install ros-noetic-amcl ros-noetic-move-base
   sudo apt-get install ros-noetic-dwa-local-planner

1. **Clone the repository**:
   ```sh
   cd ~/catkin_ws/src
   git clone https://github.com/gadf00/navigation_bot.git
   cd navigation_bot
2. **Catkin_make**:
   ```sh
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   export TURTLEBOT3_MODEL=burger
3. **Roslaunch**:
   You must open 5 terminals:
     First terminal
     ```sh
     roscore
     ```
     Second terminal
     ```sh
     roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/path/to/your/catkin_ws/src/navigation_bot/map/map.yaml
     ```
     Third terminal
     ```sh
     roslaunch turtlebot3_gazebo turtlebot3_house.launch
     ```
     Fourth terminal
     ```sh
     roslaunch navigation_bot navigation_bot.launch
     ```
     Fifth terminal (if you want to stop the navigation)
     ```sh
     rosrun navigation_bot stop_navigation.py
     ```
## File Structure

- **launch/**: Contains the launch file navigation_bot.launch to start the system.
- **msg/**: Contains custom message definitions.
- **srv/**: Contains service definitions.
- **scripts/**: Contains the Python scripts for various functionalities.
- **map/**: Contains the map files used for navigation, including:
  - **map.yaml**: The YAML file that defines the map's metadata and references the image file.
  - **map.pgm**: The image file representing the map.
- **CMakeLists.txt**: Configuration file for building the package.
- **package.xml**: Package configuration and dependencies.


