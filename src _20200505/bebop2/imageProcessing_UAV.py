import cv2
import os

def mergePics(path):#需要修改endPic
	#path="/home/sivan7/bebop_mega/效果可以_15_46"
	image_path=path+'/image'
	lens=len([lists for lists in os.listdir(image_path) if os.path.isfile(os.path.join(image_path, lists))])
	#-------------------------修改起始点-------------------#
	startPic=1380
	#endPic=250
	endPic=lens
	#----------------------------------------------------#
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
	path="/home/sivan7/bebop_mega/Apr_22_2021_10_14_56"
	mergePics(path)
	getVideo(picPath = path+'/merge/',outVideo = path+"/mergeVideo.mp4")
	

