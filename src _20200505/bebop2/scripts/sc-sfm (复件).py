# -*- coding: UTF-8 -*-
#! /usr/bin/env python 
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import torch
from imageio import imread, imsave
from scipy.misc import imresize
import numpy as np
import argparse
import matplotlib.pyplot as plt
import os
import sys
import time
import cv2

     
    


# Root directory of the project
ROOT_DIR = "/home/sivan7/文档/sc-sfm/"
# Import Mask RCNN
sys.path.append(ROOT_DIR)  # To find local version of the library
from models import DispNet, DispResNet

parser = argparse.ArgumentParser(description='Inference script for DispNet learned with \
                                 Structure from Motion Learner inference on KITTI and CityScapes Dataset',
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument("--img-height", default=1080, type=int, help="Image height")
parser.add_argument("--img-width", default=1920, type=int, help="Image width")
parser.add_argument("--no-resize", action='store_true', help="no resizing is done")
device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
args = parser.parse_known_args()[0]
args.no_resize

def mytensor2array(tensor, max_value=255, channel_first=True):
    tensor = tensor.detach().cpu()
    if max_value is None:
        max_value = tensor.max().item()
    if tensor.ndimension() == 2 or tensor.size(0) == 1:    
        array = (255*tensor.squeeze().numpy()/max_value).clip(0, 255).astype(np.uint8)   
       
        if channel_first:
            array = array.transpose(2, 0, 1)

    elif tensor.ndimension() == 3:
        assert(tensor.size(0) == 3)
        array = 0.5 + tensor.numpy()*0.5
        if not channel_first:
            array = array.transpose(1, 2, 0)
    return array

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

def sc_sfm(frame):    
    
    
    img = np.float32(frame)
    img = np.transpose(img, (2, 0, 1))
    
    tensor_img = torch.from_numpy(img).unsqueeze(0)
    #start = time.time()
    tensor_img = (tensor_img/255 - 0.5)/0.5
    #end = time.time()
    #seconds = end - start
    tensor_img = tensor_img.to(device)
    
    output = disp_net(tensor_img)[0]
    torch.cuda.synchronize()
    disp = (255*mytensor2array(output, max_value=None, channel_first=False)).astype(np.uint8)
    
    
    #print( "Time taken : {0} seconds".format(seconds))        
    return disp

def callback(data):
    
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(data, "bgr8")
    global count, picnum
    count = count + 1
    if count == 1:
        count = 0
        
        picnum += 1
        start = time.time()
        print frame.shape
        img_ori = cv2.resize(frame, (856, 480), cv2.INTER_LINEAR)
        depth = sc_sfm(img_ori)

        depth = depth.astype(np.int) 
        depth = depth + 20
        depth = np.clip(depth, 0, 255)
        depth = depth.astype(np.uint8) 

        save_ori_img = save_ori + str(picnum).zfill(6) + '.jpg'
        save_depth_img = save_depth + str(picnum).zfill(6) + '.jpg'
        save_c_depth_img = save_c_depth + str(picnum).zfill(6) + '.jpg'
        
        depth_pub = bridge.cv2_to_imgmsg(depth, encoding="mono8")
        header = Header(frame_id=save_c_depth_img)
        depth_pub.header = header
        pub.publish(depth_pub)
        
        cv2.imwrite(save_ori_img, frame)
        cv2.imwrite(save_depth_img, depth)
        #depth = cv2.imread("/home/sivan7/bebop_mega/fail_1/depth/000282.jpg",0)

        #cv2.imshow('ori', frame)
        cv2.imshow('depth', depth)

        cv2.waitKey(1)
        duration = time.time() - start
        duration_str = "time %s" % duration
        #rospy.loginfo(duration_str)
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
    
    save_ori = '/home/sivan7/bebop_mega/'+ time.strftime('%b_%d_%Y_%H_%M_%S',time.localtime(time.time())) + '/'
    save_depth = save_ori + 'depth/'
    save_c_depth = save_ori + 'c_depth/'
    save_ori = save_ori + 'image/'
    mkdir(save_ori)
    mkdir(save_depth)
    mkdir(save_c_depth)
    
    disp_net = DispResNet().to(device)
    weights = torch.load("/home/sivan7/文档/sc-sfm/finetune_dispnet_model_best.pth.tar", map_location=device)
    disp_net.load_state_dict(weights['state_dict'])
    disp_net.eval()

    #depth_tran()
    rospy.init_node('Depth_Estimation_Node', anonymous=False)
    pub = rospy.Publisher('/Controller/depth', Image, queue_size = 1)
    rospy.Subscriber("/bebop/image_raw", Image, callback)
    #rospy.Subscriber("/bebop/image_raw", Image, callback, queue_size=1, buff_size=100000000)
    #cv2.namedWindow("depth")
    rospy.spin()

    


