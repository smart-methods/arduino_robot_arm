#!/usr/bin/env python
import cv2
import numpy as np

def nothing(pos):
	pass

cap=cv2.VideoCapture(1)

cv2.namedWindow('Thresholds')
cv2.createTrackbar('LH','Thresholds',0,255, nothing)
cv2.createTrackbar('LS','Thresholds',0,255, nothing)
cv2.createTrackbar('LV','Thresholds',0,255, nothing)
cv2.createTrackbar('UH','Thresholds',255,255, nothing)
cv2.createTrackbar('US','Thresholds',255,255, nothing)
cv2.createTrackbar('UV','Thresholds',255,255, nothing)


while(1):
	_, img = cap.read()
	    
	#converting frame(img i.e BGR) to HSV (hue-saturation-value)

	hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
	
	lh=cv2.getTrackbarPos('LH','Thresholds')
	ls=cv2.getTrackbarPos('LS','Thresholds')
	lv=cv2.getTrackbarPos('LV','Thresholds')

	uh=cv2.getTrackbarPos('UH','Thresholds')
	us=cv2.getTrackbarPos('US','Thresholds')
	uv=cv2.getTrackbarPos('UV','Thresholds')
	
	#defining the Range of color
	color_lower=np.array([lh,ls,lv],np.uint8)
	color_upper=np.array([uh,us,uv],np.uint8)
	
	
	#finding the range of color in the image

	color=cv2.inRange(hsv,color_lower,color_upper)
	
	#Morphological transformation, Dilation  	
	kernal = np.ones((5 ,5), "uint8")


	color=cv2.dilate(color,kernal)
           
           
    	cv2.imshow("Color",color)
    	cv2.imshow("Original Image",img)	
    	 
	if cv2.waitKey(1)== ord('q'):
		break

cap.release()
cv2.destroyAllWindows()
