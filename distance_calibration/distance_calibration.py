from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
import time
from matplotlib import pyplot as plt
from datetime import datetime

#initialize camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 10
rawCapture = PiRGBArray(camera, size=(640,480))

#warmup
time.sleep(0.1)

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = None

target_distances_arange = np.arange(0.1,1.3,0.1)
target_distances = []
for item in target_distances_arange:
	target_distances.append(round(item,1))
print(target_distances)
heights = []
widths = []

lower_bound = np.array([52,40,31]) #lower bound for green light search
upper_bound = np.array([112,255,255]) #upper bound for green light search
index = 0

with open("distance_calibration.txt","w") as file:
	file.close()



for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		#grab current frame
		
	image = frame.array[0:220]
	image = cv2.flip(image, -1)
	
	text1 = str(target_distances[index])
	#text2 = "then press enter"

	cv2.putText(image, text1, (0,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0),2)
	#cv2.putText(image, text2, (0,75), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0),2)
	image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) #convert image to hsv for mask
		

	mask = cv2.inRange(image_hsv, lower_bound, upper_bound) #create mask based on bounds
		
	#find outer contours of mask
	contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 

		#create tracking circle and crosshairs based on largest contour
	if len(contours) > 0:
		largest_contour = max(contours, key=cv2.contourArea)
		rect = cv2.minAreaRect(largest_contour)
		box = cv2.boxPoints(rect)
		box = np.int0(box)
		cv2.drawContours(image, [box], 0, 255, 2)
	#show the frame to our screen
	cv2.imshow("Frame", image)
	
	key = cv2.waitKey(1) & 0xFF
	if key == ord("\r"):
		scr_time = datetime.now()
		scr_name = scr_time.strftime("%d-%m-%Y-%H_%M_%S")
		if len(contours) > 0:
			with open(scr_name+".txt","a") as file:
				height = round(rect[1][0])
				width = round(rect[1][1])
				heights.append(height)
				widths.append(width)
				file.write(str(target_distances[index])+","+str(width)+","+str(height)+"\n")
				file.close()
			print(str(target_distances[index])+" [m] captured")
		
			index += 1
			
	rawCapture.truncate(0)
	#press the q key to stop the stream
	if key == ord("q"):
		break	
cv2.destroyAllWindows()
i=len(heights)
plt.plot(target_distances[:i],heights, label='height')
plt.plot(target_distances[:i],widths, label='width')
plt.xlabel("target distance")
plt.legend(loc='upper right', fontsize='small', frameon=True)

plt.savefig(scr_name+".jpg")

	
	
		
