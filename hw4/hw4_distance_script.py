#import packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
from datetime import datetime
import numpy as np
import RPi.GPIO as gpio
import time



#warmup
time.sleep(0.1)

#define pin allocations
trig = 16
echo = 18

def distance():
	gpio.setmode(gpio.BOARD)
	gpio.setup(trig, gpio.OUT)
	gpio.setup(echo, gpio.IN)
	
	#ensure output has no value
	gpio.output(trig, False)
	time.sleep(0.01)
	
	#generate trigger pulse
	gpio.output(trig, True)
	time.sleep(0.00001)
	gpio.output(trig, False)
	
	#generate echo time signal
	while gpio.input(echo) == 0:
		pulse_start = time.time()
		
	while gpio.input(echo) == 1:
		pulse_end = time.time()
	
	pulse_duration = pulse_end - pulse_start
	
	#convert time to distance
	distance = pulse_duration*17150
	distance = round(distance, 2)
	
	gpio.cleanup()
	return distance

distances = []
i = 1
while i<11:
	current_distance = distance()
	distances.append(current_distance)
	print("ping ",i, " Distance: ", current_distance, "cm")
	i += 1
	time.sleep(0.1)
	
	
ave_dist = (np.mean(distances)).round(3)

#initialize camera
camera = PiCamera()
camera.resolution = (640, 480)

#define image path and file name
timestamp = datetime.now().strftime("%d-%m-%Y-%H_%M_%S")
image_path = f"/home/dzinobile/capture_{timestamp}.jpg"

#capture image
camera.capture(image_path)
print(f"Image captured and saved as {image_path}")

#define text parameters
text = "Average distance [cm]: "+str(ave_dist)
org = (50,50)
font = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 1
color = (255, 255, 255)
thickness = 3
lineType = cv2.LINE_AA

#add text and overwrite image
image = cv2.imread(image_path)
image = cv2.flip(image, -1)
cv2.putText(image, text, org, font, fontScale, color, thickness, lineType)
cv2.imwrite(image_path, image)

#display image with text
cv2.imshow("captured image", image)

#display until any key is pressed
cv2.waitKey(0)
cv2.destroyAllWindows()
