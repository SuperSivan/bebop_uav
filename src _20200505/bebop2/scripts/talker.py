# -*- coding: UTF-8 -*-
#!/usr/bin/python3.5
import rospy
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

def talker():
     pub = rospy.Publisher('/img_tr', Image, queue_size=1) 
     rospy.init_node('img_tr', anonymous=True) 
     rate = rospy.Rate(30)
     bridge = CvBridge()
     #Video = cv2.VideoCapture(0)
     Video = cv2.VideoCapture('./vedio/1.avi')
     while not rospy.is_shutdown():
         ret, img = Video.read()
         cv2.imshow("talker", img)
         cv2.waitKey(3)
         #print(img.shape)
         pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
         rate.sleep()

if __name__ == '__main__':
     try:
         talker()
     except rospy.ROSInterruptException:
         pass

