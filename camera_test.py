#!/usr/bin/env python3
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



class image_converter:

  # instance variables
  def __init__(self):
    self.image_pub = rospy.Publisher("new_image_pub",Image,queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_sub",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      
      # Let's rotate the image by 180 degrees
      cv_image = cv2.rotate(cv_image, cv2.cv2.ROTATE_180)
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    # print("number of rows:",rows)
    # print("number of columns:",cols)

    # Here's where the circle gets added
    # check out the api: 
    # https://www.geeksforgeeks.org/python-opencv-cv2-circle-method/
    cv2.circle(cv_image, (160,120), 30, (0,0,255))

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()
  
  # rosrun image_test nw_camera_test.py /image_sub:=/camera/image

if __name__ == '__main__':
    main(sys.argv)
