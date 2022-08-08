#!/usr/bin/env python3

#AUTHOR: Dani Massa
#DATE: 2/9/22
import rospy
from sensor_msgs.msg import LaserScan

def callback(LaserScan):
    print("Entry at index 0 (front):",LaserScan.ranges[0])
    print("Entry at index 90 (left):",LaserScan.ranges[90])
    print("Entry at index 180 (behind):",LaserScan.ranges[180])
    print("Entry at index 270 (right):",LaserScan.ranges[270])
    print()

def lidarScan():
    rospy.init_node('lidarScan', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback)
    rate = rospy.Rate(10)
    rospy.spin()

if __name__ == '__main__':
    try:
        lidarScan()
    except rospy.ROSInterruptException:
        pass

