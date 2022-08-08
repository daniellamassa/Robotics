#!/usr/bin/env python3

#AUTHOR: Dani Massa
#DATE: 1/19/22

import rospy
import math
from std_msgs.msg import String
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class Robot:
    def __init__(self):
        self.old_x = 0
        self.old_y = 0
        self.distance = 0
        self.heading = 0
        self.init = False
        self.move = Twist()

    def record_data(self,data):
        print("DATA is X:{:.3}, Y:{:.3}, THETA:{}".format(data.x, data.y, data.theta))
        
        if not self.init:
            print (self.init)
            self.old_x = data.x
            self.old_y = data.y
            self.heading = ((data.theta)*(180/math.pi))%360
            self.init = True
        else:
            self.distance += math.sqrt((data.x - self.old_x)**2 + (data.y - self.old_y)**2)
            self.old_x = data.x
            self.old_y = data.y
            self.heading = ((data.theta)*(180/math.pi))%360
            

        print("Distance: " + str(self.distance))
        print("Heading: " + str(self.heading))
        print()
        #DEBUGGING:
        #print(self.init)
        #print(self.move)
        
    def callback(self):
       if self.distance < 4:
          self.move.linear.x +=1
       else:
          self.move.linear.x =0
       return self.move

def deadReckon():

    rospy.init_node('deadReckon', anonymous=True)
    rospy.Subscriber("turtle1/pose",Pose, myRobot.record_data)
    pub = rospy.Publisher("turtle1/cmd_vel",Twist, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
       pub.publish(myRobot.callback())
       rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        myRobot = Robot()
        deadReckon()
    except rospy.ROSInterruptException:
        pass
