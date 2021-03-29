# encoding: utf-8
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
#from __future__ import unicode_literals

import rospy
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
import argparse
#keyboard 
import sys
import tty
import termios

import sys
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
# Root directory of the project
ROOT_DIR = "/home/zhang/Project/pysot/"
sys.path.append(ROOT_DIR)  # To find local version of the library
import time
import cv2
import torch
import numpy as np
from glob import glob

from pysot.core.config import cfg
from pysot.models.model_builder import ModelBuilder
from pysot.tracker.tracker_builder import build_tracker

torch.set_num_threads(1)

parser = argparse.ArgumentParser(description='tracking demo')
parser.add_argument('--config', type=str, help='config file')
parser.add_argument('--snapshot', type=str, help='model name')
parser.add_argument('--video_name', default='', type=str,
                    help='videos or image files')
args = parser.parse_args()


def readchar():#keyboard function
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def readkey(getchar_fn=None):#keyboard function
    getchar = getchar_fn or readchar
    c1 = getchar()
    if ord(c1) != 0x1b:
        return c1
    c2 = getchar()
    if ord(c2) != 0x5b:
        return c1
    c3 = getchar()
    return chr(0x10 + ord(c3) - 65)
def mkdir(path):
	path = path.strip()
	path = path.rstrip('\\')####

	isExists = os.path.exists(path)
	if not isExists:
		os.makedirs(path)
	return	


def callback(data):  
    global first_frame
    start = time.time()
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(data, "bgr8")
    #key=readkey()
    #while(1):
        #key=readkey()
        #cv2.imshow(video_name, frame)
        #cv2.waitKey(1)
        #rospy.loginfo("1")
        #if key=='w':
            #break

    #if bool(1-bool(keyboard.wait('a'))):
    if first_frame:
        try:
            init_rect = cv2.selectROI(video_name, frame, False, False)
        except:
            exit()
        tracker.init(frame, init_rect)
        first_frame = False
    else:
        outputs = tracker.track(frame)
        if 'polygon' in outputs:
            polygon = np.array(outputs['polygon']).astype(np.int32)
            cv2.polylines(frame, [polygon.reshape((-1, 1, 2))],
                          True, (0, 255, 0), 3)
            mask = ((outputs['mask'] > cfg.TRACK.MASK_THERSHOLD) * 255)
            mask = mask.astype(np.uint8)
            mask = np.stack([mask, mask*255, mask]).transpose(1, 2, 0)
            frame = cv2.addWeighted(frame, 0.77, mask, 0.23, -1)
        else:
            bbox = list(map(int, outputs['bbox']))
            cv2.rectangle(frame, (bbox[0], bbox[1]),
                          (bbox[0]+bbox[2], bbox[1]+bbox[3]),
                          (0, 255, 0), 3)
            
            
            score = outputs['best_score']
            display_text = '%.2f' % (score)
            font = cv2.FONT_HERSHEY_COMPLEX
            cv2.putText(frame, display_text, (bbox[0], bbox[1]+20), font, 2, (0, 255, 0), 2)
            tmp = int(score*100)
            bbox.append(tmp)
            bbox_msg = Int16MultiArray(data=bbox)    
            rospy.loginfo(bbox_msg)
            pub.publish(bbox_msg)

        img = cv2.resize(frame, img_size, interpolation=cv2.INTER_CUBIC)
        videoWriter.write(img)          
        cv2.imshow(video_name, frame)
        cv2.waitKey(1)

    duration = time.time() - start
    duration_str = "time %s" % duration
    pub_info = "I talk: [%d],[%d],[%d],[%d]" % (bbox[0], bbox[1], bbox[2], bbox[3])
    #rospy.loginfo(pub_info)
    #rospy.loginfo(duration_str)
    
def track():
    rospy.init_node('target_track', anonymous=True)
    pub = rospy.Publisher('/bebop/person_box', Int16MultiArray, queue_size = 1)
    #rospy.Subscriber("/bebop/image_raw", Image, callback)
    rospy.Subscriber("/bebop/image_raw", Image, callback, queue_size=1, buff_size=10000000)
    #rospy.Subscriber("/bebop/image_raw", Image, callback, queue_size=1, buff_size=10000000)
    rospy.spin()
    
if __name__ == '__main__':
    # load config
    config_file = "/home/sivan7/文档/pysot/experiments/siamrpn_r50_l234_dwxcorr/config.yaml"
    snapshot_file = "/home/sivan7/文档/pysot/experiments/siamrpn_r50_l234_dwxcorr/model.pth"
    cfg.merge_from_file(config_file)
    cfg.CUDA = torch.cuda.is_available()
    device = torch.device('cuda' if cfg.CUDA else 'cpu')

    # create model
    model = ModelBuilder()

    # load model
    model.load_state_dict(torch.load(snapshot_file,
        map_location=lambda storage, loc: storage.cpu()))
    model.eval().to(device)

    # build tracker
    tracker = build_tracker(model)

    global first_frame
    first_frame = True
    video_name = 'webcam'
    #cv2.namedWindow(video_name, cv2.WND_PROP_FULLSCREEN)
    
    fps = 20
    video_dir = '/home/sivan7/bebop_uav/uav_data/result' + time.strftime('%b_%d_%Y_%H_%M_%S',time.localtime(time.time())) + '.avi'
    img_size = (856,480)
    fourcc = cv2.VideoWriter_fourcc('M','J','P','G') #opencv3.0
    videoWriter = cv2.VideoWriter(video_dir, fourcc, fps, img_size)
    
    pub = rospy.Publisher('/bebop/person_box', Int16MultiArray, queue_size = 1)
    track()
