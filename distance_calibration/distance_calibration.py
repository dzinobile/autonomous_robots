import cv2
import numpy as np
from matplotlib import pyplot as plt

target_distances = np.arange(0.1,1.3,0.1)
target_distances = np.round(target_distances,1)
lower_bound_red = np.array([133,111,100])
upper_bound_red = np.array([255,255,255])
lower_bound_green = np.array([36,50,79])
upper_bound_green = np.array([73,255,255])
lower_bound_blue = np.array([74,25,0])
upper_bound_blue = np.array([114,255,255])

red_width = []
red_height = []
blue_width = []
blue_height = []
green_width = []
green_height = []


with open("distance_calibration.txt","w") as file:

	for i in range(1,13):
		img = cv2.imread("cropped/"+str(i)+"_cropped.jpg")
		img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

		red_mask = cv2.inRange(img_hsv,lower_bound_red,upper_bound_red)
		green_mask = cv2.inRange(img_hsv,lower_bound_green,upper_bound_green)
		blue_mask = cv2.inRange(img_hsv,lower_bound_blue,upper_bound_blue)

		red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

		if len(red_contours) > 0:
			largest_contour = max(red_contours, key=cv2.contourArea)
			rect = cv2.minAreaRect(largest_contour)
			rxy, rwh, ra = rect
			rw,rh = rwh
			if rh < rw:
				rw,rh = rh,rw
				rwh = (rw,rh)
			box = cv2.boxPoints(rect)
			box = np.int0(box)
			cv2.drawContours(img, [box], 0, (0,0,255), 1)
			file.write(str(target_distances[i-1])+",red,"+str(rwh)+"\n")
			red_width.append(round(rwh[0]))
			red_height.append(round(rwh[1]))

		if len(green_contours) > 0:
			largest_contour = max(green_contours, key=cv2.contourArea)
			rect = cv2.minAreaRect(largest_contour)
			gxy, gwh, ga = rect
			gw,gh = gwh
			if gh < gw:
				gw,gh = gh,gw
				gwh = (gw,gh)
			box = cv2.boxPoints(rect)
			box = np.int0(box)
			cv2.drawContours(img, [box], 0, (0,255,0), 1)
			file.write(str(target_distances[i-1])+",green,"+str(gwh)+"\n")
			green_width.append(round(gwh[0]))
			green_height.append(round(gwh[1]))
		

		if len(blue_contours) > 0:
			largest_contour = max(blue_contours, key=cv2.contourArea)
			rect = cv2.minAreaRect(largest_contour)
			bxy, bwh, ba = rect
			bw,bh = bwh
			if bh < bw:
				bw, bh = bh, bw
				bwh = (bw,bh)
			box = cv2.boxPoints(rect)
			box = np.int0(box)
			cv2.drawContours(img, [box], 0, (255,0,0), 1)
			file.write(str(target_distances[i-1])+",blue,"+str(bwh)+"\n")
			blue_width.append(round(bwh[0]))
			blue_height.append(round(bwh[1]))
		
		cv2.imwrite("boxed/"+str(i)+".jpg",img)

plt.plot(target_distances,blue_width,label='blue width')
plt.plot(target_distances,blue_height,label='blue height')
plt.plot(target_distances,red_height,label='red height')
plt.plot(target_distances,red_width,label='red width')
plt.plot(target_distances,green_height,label='green height')
plt.plot(target_distances,green_width,label='green width')
plt.legend()
plt.show()



	
	
		
