# -*- coding: UTF-8 -*-
#!/usr/bin/python3.5
import rospy
from std_msgs.msg import String
import sys
print ("Python Version {}".format(str(sys.version).replace('\n', '')))
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
print ("Python Version {}".format(str(sys.version).replace('\n', '')))
import cv_bridge
import cv2
 
def talker():
     pub = rospy.Publisher('chatter', String, queue_size=10)
     rospy.init_node('talker', anonymous=True)
     rate = rospy.Rate(10) # 10hz
     while not rospy.is_shutdown():
         src = cv2.imread('/media/sivan7/Elements SE/微信截图_20210221072316.png', 1)
         if src is None:
             print("图像为空")
         else:
             cv2.imshow('1', src)
             cv2.waitKey(0)
         rate.sleep()
 
if __name__ == '__main__':
     try:
         talker()
     except rospy.ROSInterruptException:
         pass
