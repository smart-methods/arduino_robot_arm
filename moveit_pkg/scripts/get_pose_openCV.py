#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose

import sys
import cv2
import numpy as np


cap=cv2.VideoCapture(0)


pub = rospy.Publisher('direction', Pose, queue_size=10)
rospy.init_node('vision', anonymous=True)
rate = rospy.Rate(10) # 10hz
rospy.loginfo("Direction is published ........ # see /direction topic")



positionX = -0.0023734011670250126
positionY = 0.026825106858739834
positionZ = 0.32402280551708873

orientationX = 0.4966690471541269
orientationY = -0.4966708043538031
orientationZ = -0.5033089303160498
orientationW = -0.5033071531040186


while(1):
	_, img = cap.read()
	    
	#converting frame(img i.e BGR) to HSV (hue-saturation-value)

	hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
	blue_lower=np.array([94,123,46],np.uint8)
	blue_upper=np.array([125,255,255],np.uint8)


	blue=cv2.inRange(hsv,blue_lower,blue_upper)
	
	#Morphological transformation, Dilation  	
	kernal = np.ones((5 ,5), "uint8")


	blue=cv2.dilate(blue,kernal)


	(_,contours,hierarchy)=cv2.findContours(blue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	
	if len(contours)>0:
		contour= max(contours,key=cv2.contourArea)
		area = cv2.contourArea(contour)
		if area>800: 
			x,y,w,h = cv2.boundingRect(contour)
		
			ss = 'x:' + str(x) + ', y:' + str(y)
			
			cv2.putText(img,ss,(x-20,y-5),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),1,cv2.LINE_AA)
			
			OldRangeX = (640 - 0)  
			NewRangeX = (0.2 - (-0.2))  
			positionX = (((x - 0) * NewRangeX) / OldRangeX) + (-0.2)

			OldRangeY = (480 - 0)  
			NewRangeY = (0.2 - (-0.2))  
			positionY = (((y - 0) * NewRangeY) / OldRangeY) + (-0.2)

			positionZ = 0.0087

			orientationX = 0.0
			orientationY = 0.0
			orientationZ = 0.0
			orientationW = 1.0
	
	cv2.imshow("Mask",blue)
	cv2.imshow("Color Tracking",img)
	k = cv2.waitKey(10) & 0xff # Press 'ESC' for exiting video
    	if k == 27:
		break

	
	if not rospy.is_shutdown():

		p = Pose()
		p.position.x = positionX
		p.position.y = positionY
		p.position.z = positionZ
		p.orientation.x = orientationX
		p.orientation.y = orientationY
		p.orientation.z = orientationZ
		p.orientation.w = orientationW

		pub.publish(p)
		rate.sleep()

cap.release()
cv2.destroyAllWindows()

