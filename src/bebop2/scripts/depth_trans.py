#! /usr/bin/env python 
# encoding: utf-8
import cv2
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import argparse
import roslaunch
import torch
import sys
import os
import time
from torch.autograd import Variable
import matplotlib.pyplot as plt
import numpy as np

# Root directory of the project
ROOT_DIR = "/home/sivan7/文档/MegaDepth/"

# Import Mask RCNN
sys.path.append(ROOT_DIR)  # To find local version of the library

from options.train_options import TrainOptions
opt = TrainOptions().parse()  # set CUDA_VISIBLE_DEVICES before import torch
from data.data_loader import CreateDataLoader
from models.models import create_model
from skimage import io
from skimage.transform import resize

def mkdir(path):
	path = path.strip()
	path = path.rstrip('\\')####

	isExists = os.path.exists(path)
	if not isExists:
		os.makedirs(path)
	return

def accumulate(a):
    b = np.zeros(len(a)+1)
    mean = abs(a).mean() * 2
    print mean
    for i in range(len(a)):
        if (abs(a[i]) > 2):
            b[i+1] = 0
        else:
            b[i+1] = b[i] + a[i]
    return b

def gray2rgb(im, cmap='gray'):
    cmap = plt.get_cmap(cmap)
    rgba_img = cmap(im.astype(np.float32))
    rgb_img = np.delete(rgba_img, 3, 2)
    return rgb_img

def filter(img):
    a = img.mean(0)
    c = np.diff(a)
    d = accumulate(c)
    new = img.astype(int)
    new = img + d*2
    new = np.clip(new, 0, 255)
    new = new.astype(np.uint8)
    return new

def mega(img_ori):
    #print("============================= TEST ============================")
    model.switch_to_eval()
    
    img = np.float32(img_ori)/255.0
    input_img =  torch.from_numpy( np.transpose(img, (2,0,1)) ).contiguous().float()
    input_img = input_img.unsqueeze(0)
    input_images = Variable(input_img.cuda() )
    pred_log_depth = model.netG.forward(input_images) 
    pred_log_depth = torch.squeeze(pred_log_depth)
    pred_depth = torch.exp(pred_log_depth)

    pred_inv_depth = 1/pred_depth
    pred_inv_depth = pred_inv_depth.data.cpu().numpy()
    
        
    pred_inv_depth = pred_inv_depth/np.amax(pred_inv_depth)
    pred_inv_depth = (1 -pred_inv_depth) * 255
    inv_depth_int = pred_inv_depth.astype(np.uint8) 
    inv_depth_int = inv_depth_int[0:179,:]
    depth_Hist = cv2.equalizeHist(inv_depth_int)
    depth_Hist = depth_Hist.astype(np.int) 
    #depth_Hist  = depth_Hist + 5
    depth_Hist = np.clip(depth_Hist, 0, 255)
    depth_Hist = depth_Hist.astype(np.uint8)
    depth_Hist = cv2.resize(depth_Hist, (856, 480), cv2.INTER_LINEAR)
    
    return depth_Hist

def callback(data):
    
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(data, "bgr8")
    global count, picnum
    count = count + 1
    if count == 1:
        count = 0
        
        picnum += 1
        start = time.time()
        
        img_ori = cv2.resize(frame, (320, 240), cv2.INTER_LINEAR)
        depth = mega(img_ori)

        save_ori_img = save_ori + str(picnum).zfill(6) + '.jpg'
        save_depth_img = save_depth + str(picnum).zfill(6) + '.jpg'
        save_c_depth_img = save_c_depth + str(picnum).zfill(6) + '.jpg'
        cv2.imwrite(save_ori_img, frame)
        cv2.imwrite(save_depth_img, depth)
        #depth = cv2.imread("/home/zhang/bebop_mega/fail_1/depth/000282.jpg",0)
        depth_pub = bridge.cv2_to_imgmsg(depth, encoding="mono8")
        header = Header(frame_id=save_c_depth_img)
        depth_pub.header = header
        pub.publish(depth_pub)

        cv2.imshow('ori', frame)
        cv2.imshow('depth', depth)

        cv2.waitKey(1)
        duration = time.time() - start
        duration_str = "time %s" % duration
        rospy.loginfo(duration_str)
        #print "all time"
        #print (end-start)
    else:
        pass
        
def depth_tran():
    global count
    count = 0
    rospy.init_node('depth_trans', anonymous=True)
    pub = rospy.Publisher('/drone/depth', Image, queue_size = 1)
    #rospy.Subscriber("/bebop/image_raw", Image, callback)
    rospy.Subscriber("/bebop/image_raw", Image, callback, queue_size=1, buff_size=10000000)
    rospy.spin()
    
if __name__ == '__main__':


    global picnum, count
    picnum = 0
    count = 0

    input_height = 240#384
    input_width  = 320#512
    
    save_ori = '/home/sivan7/bebop_mega/'+ time.strftime('%b_%d_%Y_%H_%M_%S',time.localtime(time.time())) + '/'
    save_depth = save_ori + 'depth/'
    save_c_depth = save_ori + 'c_depth/'
    save_ori = save_ori + 'image/'
    mkdir(save_ori)
    mkdir(save_depth)
    mkdir(save_c_depth)

    count = 0
    
    model = create_model(opt)
    model.switch_to_eval()
    #depth_tran()
    
    global count
    count = 0
    rospy.init_node('Depth_Estimation_Node', anonymous=False)
    pub = rospy.Publisher('/Controller/depth', Image, queue_size = 1)
    #rospy.Subscriber("/bebop/image_raw", Image, callback)
    rospy.Subscriber("/bebop/refine", Image, callback, queue_size=1, buff_size=100000000)
    #cv2.namedWindow("depth")
    rospy.spin()

    


