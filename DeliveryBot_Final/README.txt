# Final Project CSC-325 Winter 2022
Tyler Nass & Dani Massa

This project contains code using ROS which uses a Turtlebot3 to simulate a delivery service in ISEC 178 and 180. 

Usage:

1) SETUP

	- run roscore on the host computer & boot up the Turtlebot3
	- ssh into the Turtlebot
	- run the bringup command:
		- roslaunch turtlebot3_bringup turtlebot3_robot.launch
	- make sure the robot is within the bounds of the map (ISEC 178 & 180 + the hallway outside these rooms) before running rviz
	- launch rviz with the map (assuming the final_project_csc325 package is in the catkin workspace in the home directory):
		- roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/catkin_ws/src/final_project_csc325/src/final_map.yaml

2) LOCALIZATION

	- open the teleop keyboard:
		- roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
	- drive the robot around until the green arrows in the rviz screen converge on the correct location of the turtlebot

3) RUNNING THE PROGRAM

	- run the program using the command:
		- rosrun final_project_csc325 final_project.py
