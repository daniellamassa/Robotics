#!/usr/bin/env python3

#AUTHOR: Dani Massa
#DATE: 1/29/22
import rospy,math
from std_msgs.msg import String
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class Robot:

    def __init__(self):
        self.init = False
        self.turtlesim_pose = Pose()


    def poseCallback(self,data):
        self.turtlesim_pose.x = data.x
        self.turtlesim_pose.y = data.y
        self.turtlesim_pose.theta = data.theta
        
    def drive_square(self, length_of_side):
        rate = rospy.Rate(10)
        number_of_sides = 4
        current_side = 1
        print ("Starting the square")
        while current_side <= number_of_sides:
            print("Driving side",current_side)
            move(length_of_side, True)
            print("Rotating")
            rotate(degrees2radians(90), True)
            rate.sleep()
            current_side+=1
        

    def setDesiredOrientation(self,desired_angle_radians):
        print ("Setting orientation to:",desired_angle_radians)
        print ("Current angle is:",self.turtlesim_pose.theta)
        
        relative_angle_radians = desired_angle_radians - self.turtlesim_pose.theta
        rotate(relative_angle_radians,True)


    def moveGoal(self,goal_pose, distance_tolerance):
        move_publish = rospy.Publisher("turtle1/cmd_vel",Twist, queue_size = 10)
        rate = rospy.Rate(10)
        vel_msg = Twist()
        distance_from_goal = findDistance(goal_pose.x, self.turtlesim_pose.x, goal_pose.y, self.turtlesim_pose.y)
        
        while (distance_from_goal > distance_tolerance):
            rotate_amount = math.atan2((goal_pose.y - self.turtlesim_pose.y),(goal_pose.x - self.turtlesim_pose.x))
            vel_msg.linear.x = 1.5 * distance_from_goal
            vel_msg.angular.z = 4 * (rotate_amount - self.turtlesim_pose.theta)
            move_publish.publish(vel_msg)
            rate.sleep()
            distance_from_goal = findDistance(goal_pose.x, self.turtlesim_pose.x, goal_pose.y, self.turtlesim_pose.y)
            
            if abs(vel_msg.angular.z) < 0.2:
               rotate_amount = 0
               
        rate.sleep()


def findDistance(x1,x0,y1,y0):
    newDistance = math.sqrt((x1-x0)**2 + (y1-y0)**2)
    return newDistance


def degrees2radians(angle):
	return angle * (math.pi/180.0)


def move(distance, isForward):

    move_publish = rospy.Publisher("turtle1/cmd_vel",Twist, queue_size=10)
    outData = Twist()
    t0 = rospy.get_rostime().secs

    while t0 == 0:
        t0 = rospy.get_rostime().secs

    current_distance = 0
    rate = rospy.Rate(10)

    if isForward == True:
       outData.linear.x = 1.0
    else:
       outData.linear.x = -1.0
    while current_distance < distance:
        move_publish.publish(outData)
        t1 = rospy.get_rostime().secs
        current_distance = abs(outData.linear.x * (t1 - t0))
        rate.sleep()
       

def rotate(relative_angle, isClockwise):

    rotate_publish = rospy.Publisher("turtle1/cmd_vel",Twist, queue_size=10)
    outData = Twist()
    t0 = rospy.get_rostime().secs

    while t0 == 0:
        t0 = rospy.get_rostime().secs

    current_angle = 0
    rate = rospy.Rate(10)

    if isClockwise == True:
       outData.angular.z = degrees2radians(30.0)
    else:
       outData.angular.z = degrees2radians(-30.0)
    while abs(current_angle) < abs(relative_angle):
        rotate_publish.publish(outData)
        t1 = rospy.get_rostime().secs
        current_angle = abs(outData.angular.z * (t1 - t0))
        rate.sleep()
       

def moveRobot():

    rospy.init_node('squareMover', anonymous=True)
    rospy.Subscriber("/turtle1/pose", Pose, myRobot.poseCallback)
    rate = rospy.Rate(100)

    #move(3.0, False)
    #rate.sleep()

    #rotate(degrees2radians(180.0),True)
    #rate.sleep()
   
    #myRobot.setDesiredOrientation(degrees2radians(270))
    #rate.sleep()

    #goal_pose = Pose()
    #goal_pose.x = 1.0
    #goal_pose.y = 1.0
    #goal_pose.theta = 0.0
    #myRobot.moveGoal(goal_pose,0.1)
    #rate.sleep()
    
    myRobot.drive_square(2)
    rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    myRobot = Robot()
    moveRobot()
