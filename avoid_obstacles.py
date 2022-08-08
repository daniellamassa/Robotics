#!/usr/bin/env python3

#AUTHOR: Dani Massa
#DATE: 2/9/22
import rospy, math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
start = True


class Robot:

    def __init__(self):
        self.distanceTolerance = 0.6
        self.lidarFrontValue = 1.1
        self.outData = Twist()
        
    	
    def avoidObstacles(self):
        rospy.init_node('avoidObstacles', anonymous=True)
        rospy.Subscriber("/scan", LaserScan, myRobot.callback)
        move_publish = rospy.Publisher("/cmd_vel",Twist, queue_size=10)
        rotate_publish = rospy.Publisher("/cmd_vel",Twist, queue_size=10)
        rate = rospy.Rate(0.1)
        
        while start == True:
              if self.lidarFrontValue >= self.distanceTolerance:
                  self.outData.linear.x = 0.2
                  move_publish.publish(self.outData)
              else: 
                  self.rotate(degrees2radians(90),True)
                  rotate_publish.publish(self.outData)
              
    def callback(self,LaserScan):
        rospy.init_node('avoidObstacles', anonymous=True)
        self.lidarFrontValue = LaserScan.ranges[0]
        
        if self.lidarFrontValue == 0: #float('inf')
           self.lidarFrontValue = 1.1
          
        print("front",self.lidarFrontValue)
        print("left",LaserScan.ranges[90])
        print("behind",LaserScan.ranges[180])
        print("right",LaserScan.ranges[270])

    def lidarScan(self):
        rospy.init_node('lidarScan', anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.callback)
        rate = rospy.Rate(10)
        rospy.spin()
        
    def rotate(self,relative_angle, isClockwise):
        rotate_publish = rospy.Publisher("/cmd_vel",Twist, queue_size=10)
        outData = Twist()
        t0 = rospy.get_rostime().secs

        while t0 == 0:
              t0 = rospy.get_rostime().secs

        current_angle = 0
        rate = rospy.Rate(5)

        if isClockwise == True:
           outData.angular.z = degrees2radians(30.0)
        else:
            outData.angular.z = degrees2radians(-30.0)
        while abs(current_angle) < abs(relative_angle):
              rotate_publish.publish(outData)
              t1 = rospy.get_rostime().secs
              current_angle = abs(outData.angular.z * (t1 - t0))
              rate.sleep()


def degrees2radians(angle):
	return angle * (math.pi/180.0)
    

if __name__ == '__main__':
    myRobot = Robot()
    myRobot.avoidObstacles()

 
