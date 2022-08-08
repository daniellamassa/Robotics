#!/usr/bin/env python3

#AUTHOR: Dani Massa
#DATE: 2/9/22
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def callback(Odometry):
    quaternion_list = []
    quaternion_list.append(Odometry.pose.pose.orientation.x)
    quaternion_list.append(Odometry.pose.pose.orientation.y)
    quaternion_list.append(Odometry.pose.pose.orientation.z)
    quaternion_list.append(Odometry.pose.pose.orientation.w)
    
    euler_values = euler_from_quaternion(quaternion_list) #(roll,pitch,yaw)
    print("x position:",Odometry.pose.pose.position.x)
    print("y position:",Odometry.pose.pose.position.y)
    print("orientation:",euler_values[2])
    

def odomReader():
    rospy.init_node('odomReader', anonymous=True)
    rospy.Subscriber("/odom", Odometry, callback)
    rate = rospy.Rate(10)
    rospy.spin()

if __name__ == '__main__':
    try:
        odomReader()
    except rospy.ROSInterruptException:
        pass
