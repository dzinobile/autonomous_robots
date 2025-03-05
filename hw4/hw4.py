#import packages
import cv2
import numpy as np
from matplotlib import pyplot as plt
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
from datetime import datetime

#initialize camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 10
rawCapture = PiRGBArray(camera, size=(640,480))

#warmup
time.sleep(0.1)

#initialize lists for recorded times and processing times
rec_times = []
delta_times = []


#function for finding closest and furthest points in a list
def find_closest_and_furthest(points):
  distances = []
  for i in range(0, len(points)): 
    for j in range(i+1, len(points)): #avoid repeating pairs
      dist = np.linalg.norm(np.array(points[i])-np.array(points[j])) #find distances
      distances.append((dist, points[i], points[j])) #list distances and coordinates
  distances.sort() #sort from closest to furthest
  
  closest = distances[0][1], distances[0][2]
  furthest = distances[-1][1], distances[-1][2]
  return closest, furthest
 
#function for finding angle and orientation based on points
def find_angle(input_corners):
	corners = np.intp(input_corners) #convert values to intergers
	
	points = [] #list of all 7 points
	
	#convert array to list of tuples
	for i in corners:
		(x,y) = i.ravel()
		points.append((x,y)) 
		
	closest = [] #list of the two pairs of points that are closest
	
	#find the two pairs of closest points, add to closest list and remove from points list
	while len(points) > 3:
		c = find_closest_and_furthest(points)[0]
		closest.append(c[0])
		closest.append(c[1])
		points.remove(closest[-1])
		points.remove(closest[-2])
	
	#of the three remaining points, find the two that are closest and remove from points list
	#this leaves just the point at the tip of the arrow
	c1, c2 = find_closest_and_furthest(points)[0]
	points.remove(c1)
	points.remove(c2)
	
	#create line from the tip of one barb of the arrow to the other
	a_start, a_end = find_closest_and_furthest(closest)[1]
	
	xm, ym = (a_start[0]+a_end[0])/2, (a_start[1]+a_end[1])/2 # midpoint of line from barb to barb
	xp, yp = points[0][0], points[0][1] #x and y coordinates of tip of arrow
	
	#find angle of direction of arrow
	degrees = np.rad2deg(np.arctan2(yp-ym, xp-xm))
	degrees = -1*degrees #multiply by -1 to account for flipped image
	degrees = int((degrees + 360) % 360) #account for negative angles
	
	#determine orientation based on angle
	if 0 <= degrees < 45:
		orientation = "right"
	elif 45 <= degrees < 135:
		orientation = "up"
	elif 135 <= degrees < 225:
		orientation = "left"
	elif 225 <= degrees < 315:
		orientation = "down"
	elif 315 <= degrees < 360:
		orientation = "right"
	
	degrees = str(degrees) #convert degrees to string for display
	return corners, degrees, orientation #return list of tuples of corners, angle, and orientation
	
		
	

#video writier initialization
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = None
recording = False
with open("hw4data.txt", "w") as file: #open processing time text file for writing
	#keep looping
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		
		#grab current frame
		image = frame.array
		image = cv2.flip(image, -1) #flip orientation
		mask = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) #convert to HSV
		
		#mask image based on range chosen with colorpicker
		lower_bound = np.array([40, 30, 36]) 
		upper_bound = np.array([67, 255, 255]) 
		mask = cv2.inRange(mask, lower_bound, upper_bound)
		
		mask = cv2.GaussianBlur(mask, (15,15), 0) #blur image with 15x15 gaussian kernel
		corners = cv2.goodFeaturesToTrack(mask, 7, 0.01, 10) #find 7 corners
		
		#default degrees and orientation values
		degrees = "----" 
		orientation = "----"
		
		#if 7 corners found, display angle, orientation, and corners locations
		if corners is not None and len(corners) == 7:
			corners, degrees, orientation = find_angle(corners)
			for i in corners:
				x, y = i.ravel()
				cv2.circle(image, (x, y), 5, (255, 0, 0), -1) #place circles on corners
				
		#disiplay angle and orientation
		cv2.rectangle(image, (20, 20), (620, 100), (255, 255, 255), -1) 
		text1 = ("angle: "+ degrees + " degrees")
		text2 = ("orientation: " + orientation)
		org1 = (30,50)
		org2 = (30,90)
		font = cv2.FONT_HERSHEY_SIMPLEX
		fontScale = 1
		color = (0, 0, 0)
		thickness = 2
		lineType = cv2.LINE_AA
		cv2.putText(image, text1, org1, font, fontScale, color, thickness, lineType)
		cv2.putText(image, text2, org2, font, fontScale, color, thickness, lineType)
		
		#show the frame to our screen
		cv2.imshow("Frame", image)
		key = cv2.waitKey(1) & 0xFF
		
		if key == ord("r") and not recording: #start recording
			recording = True
			rec_start = datetime.now()
			rec_start_string = rec_start.strftime("%d-%m-%Y-%H_%M_%S")
			out = cv2.VideoWriter(rec_start_string+'.avi', fourcc, 10, (640,480))
			rec_times.append(0) #first record time value is 0
			print("recording started at "+rec_start_string)
		
		if key == ord("t") and recording: #stop recording
			recording = False
			out.release()
			out = None
			end_time_string = now.strftime("%d-%m-%Y-%H_%M_%S")
			print("recording stopped at "+end_time_string)
			break
			
		if recording:
			out.write(image)
			now = datetime.now()
			rec_time = (now-rec_start).total_seconds()
			rec_times.append(rec_time) #append current rec time to list
			delta_time = str(rec_times[1] - rec_times[0])+"\n" #find processing time between current and previous frames
			file.write(delta_time) #write processing time to text file
			rec_times.pop(0) #remove previous rec time from queue, no longer needed
			print(f"Elapsed time: {rec_time} seconds")
			
			
		
		
		#clear the stream for next frame
		rawCapture.truncate(0)
		
		#press the q key to stop the stream
		if key == ord("q"):
			break
			
#load values from text file into list		
with open("hw4data.txt","r") as file:
	for line in file:
		#convert string of second values to float of milliseconds
		delta_times.append((float(line.strip()))*1000)

#display plot and histogram 		
plot_x = np.arange(0, len(delta_times),1)
fig, axs = plt.subplots(2)
plt.suptitle("Object Tracking: Processing Time")

axs[0].plot(plot_x, delta_times, linestyle='-')
axs[0].set(xlabel="Frame",ylabel="Tracking Time [ms]")

axs[1].hist(delta_times, 50)
axs[1].set(xlabel="Processing Time [ms]", ylabel="Number of Frames")

plt.subplots_adjust(hspace=0.5)
plt.show()

#ensure recording has stopped and close all
if recording:
	out.release()
cv2.destroyAllWindows()

