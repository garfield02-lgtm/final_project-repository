Note: this package is developed for ROS2 Jazzy and ubuntu 22.04, and has not been validated or built against ROS2 Humble and associated packages. 

To run this package : 

1) initialize the turtlebot3 house simulation via the gazebo package using :
     ```export TURTLEBOT3_MODEL=waffle && ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py use_sim_time:=true```
2) in a seperate terminal, launch the turtlebot3 navigation2 package via : 
    ``` export TURTLEBOT3_MODEL=waffle && ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/house.yaml```
3) in a  third window, launch the waypoint publishing node using : 
  ``` ros2 run final_project-repository publish_waypoint ```
4) once launched, wait until the robot reaches the safe position, and then input the room name as a single word, following the naming convention listed in utils/room_waypoints.yaml
