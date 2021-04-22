import cv2
import numpy as np
 
# 图片路径
img = cv2.imread('/home/sivan7/bebop_mega/Apr_22_撞树/depth/000553.jpg',cv2.IMREAD_GRAYSCALE)
#img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#print("shape is ",img.shape)
#print("val is ",255-img[369][159])
#print("val11 is ",img.at<uchar>(369, 159))
a = []
b = []
 
 
def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        xy = "%d,%d" % (x, y)
        a.append(x)
        b.append(y)
        cv2.circle(img, (x, y), 1, (0, 0, 255), thickness=-1)
        #cv2.putText(img, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,1.0, (0, 0, 0), thickness=1)
        cv2.imshow("image", img)
        print("position is ", x,y)
        print("grey val is ",255-img[x][y])
cv2.namedWindow("image")
cv2.setMouseCallback("image", on_EVENT_LBUTTONDOWN)
cv2.imshow("image", img)
cv2.waitKey(0)
 