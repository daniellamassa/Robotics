#!/usr/bin/env python3

# final project CSC325
# AUTHOR: Dani Massa + Tyler Nass
# DATE: 3/6/22
# Run with:
# roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map_attempt_04.yaml
# python3 ~/catkin_ws/src/final_project_csc325/src/finalTest2.py

from __future__ import print_function
import sys
import math
import rospy
from std_msgs.msg import String
from turtlesim.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
import os
import numpy as np
import cv2
import roslib
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


MIN_DESTINATIONS = 1
MAX_DESTINATIONS = 2

list_of_dests = [("Room 1 Bottom Left",-0.8,4.0),("Room 1 Bottom Right",2.0,4.0),("Room 1 Top Left",1.1,11.3), ("Room 1 Top Right", 3.6, 9.7),("Room 2", 8.7, 4.4)]
start_pos = (-0.5,0.5)

class DeliveryRobot():
    def __init__(self):

        self.goal_sent = False
        rospy.on_shutdown(self.shutdown)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")
        self.move_base.wait_for_server(rospy.Duration(5))
        
        self.x = 0
        self.y = 0
        self.x2 = 0
        self.y2 = 0
        self.num_of_dests = ''
        
    def user_input(self):
    	## PROMPTS USER INPUT.

        self.num_of_dests = input("enter number of destinations (integer between " + str(MIN_DESTINATIONS) + " and " + str(MAX_DESTINATIONS) + "): ")
        # checks whether self.num_of_dests can be cast to an int & if so casts it to int
        try:
            self.num_of_dests = int(self.num_of_dests)
        except:
            # input could not be converted to an int, exit program
            print("ERROR: not a number")
            sys.exit(1)
            
        if self.num_of_dests < MIN_DESTINATIONS or self.num_of_dests > MAX_DESTINATIONS:
            # input is outside of valid range, exit program
            print("ERROR: number outside valid range")
            sys.exit(1)
        print("input is: " + str(self.num_of_dests))
        if self.num_of_dests == 1:
            prompt_string = "enter name of destination\n"
            # prints available options for destination selection <-- 1 destination
            for destination in list_of_dests:
                prompt_string += destination[0] + "\n"
            # gets user input for selected destination, and sets coordinates accordingly
            first_dest = input(prompt_string)
            for destination in list_of_dests:
                if first_dest == destination[0]:
                    self.x = destination[1]
                    self.y = destination[2]
                    
        if self.num_of_dests == 2:
            output = ""
            # prints available option for destination selection <-- 2 destinations
            for destination in list_of_dests:
                output += destination[0]
                output += "\n"
            # gets user input for first destination
            first_dest = input("enter name of first destination\n" + output)
            output = ""
            # remove selected destination 1 from list and prints remaining options for destination 2
            for destination in list_of_dests:
                if (first_dest != destination[0]):
                    output += destination[0]
                    output += "\n"
            # gets user input for second destination
            second_dest = input("enter name of second destination\n" + output)
            # sets coordinates for destination 1 and 2
            for destination in list_of_dests:
                if first_dest == destination[0]:
                    self.x = destination[1]
                    self.y = destination[2]
            for destination in list_of_dests:
                if second_dest == destination[0]:
                    self.x2 = destination[1]
                    self.y2 = destination[2]
                    

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
    
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))
	    # Start moving
        self.move_base.send_goal(goal)

	    # Allow TurtleBot up to 180 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(180)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)
    
    def position_movement(self,x,y):
        # rospy.init_node('nav_test', anonymous=False)
        navigator = DeliveryRobot()

        # Customize the following values so they are appropriate for your location
        position = {'x': x, 'y' : y}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = navigator.goto(position, quaternion)

        if success:
            rospy.loginfo("Hooray, reached the desired pose")
        else:
            rospy.loginfo("The base failed to reach the desired pose")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('nav_test', anonymous=False)
    Robot = DeliveryRobot()
    # try:
    Robot.position_movement(start_pos[0],start_pos[1])
    Robot.user_input()
        
        ## MOVE TURTLEBOT 
    Robot.position_movement(Robot.x,Robot.y)
    rospy.sleep(4)
    if Robot.num_of_dests == 2:
        Robot.position_movement(Robot.x2,Robot.y2)         

   # except:
   #     print()
   #     rospy.loginfo("Ctrl-C caught. Quitting")
        
