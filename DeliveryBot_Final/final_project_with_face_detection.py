#!/usr/bin/env python3

# final project CSC325
# AUTHOR: Dani Massa + Tyler Nass
# DATE: 3/6/22
# Run with:
# python3 ~/catkin_ws/src/final_project_csc325/src/finalTest.py

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


face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_eye.xml')


MIN_DESTINATIONS = 1
MAX_DESTINATIONS = 2

list_of_dests = [("corner",2.0,0.2),("notch",2.1,3.2),("pole",-4.7,0.8)]
start_pos = (2.0,0.2)
image_name_index = 0
image_name_list = ["dest1_face", "dest2_face"]

class DeliveryRobot():
    def __init__(self):

        self.goal_sent = False
        rospy.on_shutdown(self.shutdown)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")
        self.move_base.wait_for_server(rospy.Duration(5))
        
        self.at_destination = False
        self.face_detected = False
        
        self.image_pub = rospy.Publisher("new_image_pub",Image,queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image",Image,self.callback)
        self.image = 0
        
    def user_input():
        position_movement(start_pos[0],start_pos[1])
        num_of_dests = input("enter number of destinations (integer between " + str(MIN_DESTINATIONS) + " and " + str(MAX_DESTINATIONS) + "): ")
        # checks whether num_of_dests can be cast to an int & if so casts it to int
        try:
            num_of_dests = int(num_of_dests)
        except:
            # input could not be converted to an int, exit program
            print("ERROR: not a number")
            sys.exit(1)
            
        if num_of_dests < MIN_DESTINATIONS or num_of_dests > MAX_DESTINATIONS:
            # input is outside of valid range, exit program
            print("ERROR: number outside valid range")
            sys.exit(1)
        print("input is: " + str(num_of_dests))
        if num_of_dests == 1:
            prompt_string = "enter name of destination\n"
            for destination in list_of_dests:
                prompt_string += destination[0] + "\n"
            first_dest = input(prompt_string)
            for destination in list_of_dests:
                if first_dest == destination[0]:
                    x = destination[1]
                    y = destination[2]
        if num_of_dests == 2:
            output = ""
            for destination in list_of_dests:
                output += destination[0]
                output += "\n"
            first_dest = input("enter name of first destination\n" + output)
            output = ""
            for destination in list_of_dests:
                if (first_dest != destination[0]):
                    output += destination[0]
                    output += "\n"
            second_dest = input("enter name of second destination\n" + output)
            for destination in list_of_dests:
                if first_dest == destination[0]:
                    x = destination[1]
                    y = destination[2]
            for destination in list_of_dests:
                if second_dest == destination[0]:
                    x2 = destination[1]
                    y2 = destination[2]
                    
            ## MOVE TURTLEBOT AND CHECK FOR FACES 
            # position_movement(x,y)
          
            if num_of_dests == 2:
                self.face_detected = False
                image_name_index = 1
                # position_movement(x2,y2)         


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

	    # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60)) 

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
        self.at_destination = False
        rospy.init_node('nav_test', anonymous=False)
        navigator = DeliveryRobot()

        # Customize the following values so they are appropriate for your location
        position = {'x': x, 'y' : y}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = navigator.goto(position, quaternion)

        if success:
            rospy.loginfo("Hooray, reached the desired pose")
            self.at_destination = True
        else:
            rospy.loginfo("The base failed to reach the desired pose")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
          
            cv_image = cv2.rotate(cv_image, cv2.cv2.ROTATE_180)
            self.image = cv_image
        except CvBridgeError as e:
            print(e)


    def detect_face(self):
        while True:
            faces = face_cascade.detectMultiScale(self.image, 1.1, 4)
            if len(faces) >= 1 and self.at_destination == True:
                self.face_detected = True
                for (x, y, w, h) in faces:
                    cv2.rectangle(self.image, (x, y), (x+w, y+h), (255, 0, 0), 2)
                cv2.imwrite(image_name_list[image_name_index], self.image)
            # cv2.imshow('img', self.image)
            k = cv2.waitKey(30) & 0xff
            if k==27:
                break

    

if __name__ == '__main__':
    rospy.init_node('image_converter', anonymous=True)
    Robot = DeliveryRobot()
    try:
        Robot.detect_face()  
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
        
   
'''      
    except:
        print()
        rospy.loginfo("Ctrl-C caught. Quitting")
'''
        
