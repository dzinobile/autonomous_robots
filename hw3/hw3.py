#import packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
from datetime import datetime
import numpy as np
from matplotlib import pyplot as plt

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


#video writier initialization
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = None
recording = False


with open("hw3data.txt", "w") as file: #open and overwrite hw3data.txt file
	#keep looping the camera capture
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		
		#grab current frame
		image = frame.array
		image = cv2.flip(image, -1) #camera upside down, image flip required
		image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) #convert image to hsv for mask
		
		lower_bound = np.array([40, 65, 40]) #lower bound for green light search
		upper_bound = np.array([80, 255, 255]) #upper bound for green light search
		mask = cv2.inRange(image_hsv, lower_bound, upper_bound) #create mask based on bounds
		
		#find outer contours of mask
		contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
		
		#set color and thickness of tracking circle and crosshairs
		color = (255, 255, 255)
		thickness = 2
		
		#create tracking circle and crosshairs based on largest contour
		if len(contours) > 0:
			largest_contour = max(contours, key=cv2.contourArea)
			(x, y), r = cv2.minEnclosingCircle(largest_contour)
			center = (int(x), int(y))
			radius = int(r)
			cv2.circle(image, center, radius, color, thickness)
			cv2.line(image, ((int(x)),(int(y+(r/4)))), ((int(x)),(int(y-(r/4)))), color, thickness)
			cv2.line(image, ((int(x+(r/4))),(int(y))), ((int(x-(r/4))),(int(y))), color, thickness)
		
		#show the frame to our screen
		cv2.imshow("Frame", image)
		key = cv2.waitKey(1) & 0xFF
		
		
		#press r to start recording
		if key == ord("r") and not recording: 
			recording = True
			rec_start = datetime.now()
			rec_start_string = rec_start.strftime("%d-%m-%Y-%H_%M_%S")
			out = cv2.VideoWriter(rec_start_string+'.avi', fourcc, 10, (640,480))
			rec_times.append(0) #first record time value is 0
			print("recording started at "+rec_start_string)
			
		#press t to stop recording
		if key == ord("t") and recording: 
			recording = False
			out.release()
			out = None
			end_time_string = now.strftime("%d-%m-%Y-%H_%M_%S")
			print("recording stopped at "+end_time_string)
			break #break loop once recording is stopped
		
		#save each frame while recording	
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
with open("hw3data.txt","r") as file:
	for line in file:
		#convert string of second values to float of milliseconds
		delta_times.append((float(line.strip()))*1000)

#plot processing time per frame		
plot_x = np.arange(0, len(delta_times),1)
plt.plot(plot_x, delta_times, linestyle='-')
plt.title("Object Tracking: Processing Time")
plt.xlabel("Frame")
plt.ylabel("Tracking Time [ms]")
plt.show()

#plot histogram of processing time
plt.hist(delta_times, 50)
plt.title("Object Tracking: Processing Time")
plt.xlabel("Processing Time [ms]")
plt.ylabel("Number of Frames")
plt.show()

#stop recording if not already stopped
if recording:
	out.release()
cv2.destroyAllWindows()
