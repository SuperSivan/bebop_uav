import cv2
import os

def mergePics(path,startPic=1):#需要修改endPic
	#path="/home/sivan7/bebop_mega/效果可以_15_46"
	lens=len([lists for lists in os.listdir(path) if os.path.isfile(os.path.join(path, lists))])
	endPic=80
	#endPic=lens
	print(path+'/merge')
	# if not os.path.exists(os.path.join(path,'/merge')):
	# 	os.makedirs(os.path.join(path,'/merge'))
	if not os.path.exists(path+'/merge'):
		os.makedirs(path+'/merge')
	for i in range(startPic,endPic):
	     #print(i)
	     img_image =cv2.imread(path+"/image/"+str(i).zfill(6)+".jpg")
	     img_image=cv2.resize(img_image,(570,320))
	     img_depth =cv2.imread(path+"/depth/"+str(i).zfill(6)+".jpg")
	     img_depth=cv2.resize(img_depth,(570,320))
	     img_c_depth =cv2.imread(path+"/c_depth/"+str(i).zfill(6)+".jpg")
	     img_c_depth=cv2.resize(img_c_depth,(570,320))
	     img=cv2.vconcat([img_image,img_depth,img_c_depth])#垂直拼接
	     cv2.imwrite(path+"/merge/"+str(i).zfill(6)+".jpg",img)
	     #cv2.imshow("Image",img)
	     #cv2.waitKey(5000)
	print("Merge done!")


def getVideo(picPath,outVideo,fps=14):

	files = os.listdir(picPath)
	files.sort()
	#print(files)
	fourcc = cv2.VideoWriter_fourcc(*'mp4v')
	video_writer = cv2.VideoWriter(filename=outVideo, fourcc=fourcc, fps=fps, frameSize=(570, 960)) 
	for file in files:
		#print(root)
		#获取文件路径
		#print(os.path.join(root,file))
		img = cv2.imread(os.path.join(picPath,file))
		video_writer.write(img)
	video_writer.release()
	print("Video done!")


if __name__ == '__main__':
	mergePics(path = "/home/sivan7/bebop_mega/Apr_21_2021_09_45_08")
	getVideo(picPath = '/home/sivan7/bebop_mega/Apr_21_2021_09_45_08/merge/',outVideo = "/home/sivan7/bebop_mega/Apr_21_2021_09_45_08/mergeVideo.mp4")
	

