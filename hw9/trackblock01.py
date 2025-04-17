from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
import time
import RPi.GPIO as gpio
import serial
import threading
import signal
import sys
import datetime
from datetime import datetime


#TRACKER ---------------------------------------------------------------
object_detected = False
x = 320
y = 240
rotate_target = 0

def object_tracker():
	
	global object_detected
	global rotate_target
	global x
	global y

	
	camera = PiCamera()
	camera.resolution = (640, 480)
	camera.framerate = 12
	rawCapture = PiRGBArray(camera, size=(640,480))
	out = None
	recording = False
	
	#warmup
	time.sleep(0.1)
	fourcc = cv2.VideoWriter_fourcc(*'XVID')
	out = None
	
	lower_bound = np.array([74,25,0]) #lower bound for green light search
	upper_bound = np.array([114,255,255]) #upper bound for green light search
	
	
	
	
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		image = frame.array
		image = cv2.flip(image, -1)
		image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(image_hsv, lower_bound, upper_bound) 
		contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	
		if len(contours) > 0:
			object_detected = True
			largest_contour = max(contours, key=cv2.contourArea)
			rect = cv2.minAreaRect(largest_contour)
			(x,y),box_dimensions,angle = rect
			rotate_target = round((320-x)*0.061)
			box = cv2.boxPoints(rect)
			box = np.int0(box)
			cv2.drawContours(image, [box], 0, (0,0,0), 2)
		else:
			object_detected = False
			
		if -2 < rotate_target < 2:
			cv2.putText(image,"Ready to retrieve",(0,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),2)

			
		else:
			target_text = "Rotate "+str(rotate_target)+" degrees"
			cv2.putText(image,target_text,(0,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),2)
		
		cv2.imshow("Frame", image)
		key = cv2.waitKey(1) & 0xFF
		rawCapture.truncate(0)
		
		if key == ord("r") and not recording: #start recording
			recording = True
			rec_start = datetime.now()
			rec_start_string = rec_start.strftime("%d-%m-%Y-%H_%M_%S")
			out = cv2.VideoWriter(rec_start_string+'.avi', fourcc, 12, (640,480))
			print("recording started at "+rec_start_string)
			
		if key == ord("t") and recording: #stop recording
			recording = False
			out.release()
			out = None
			print("recording stopped")
		
		if recording:
			out.write(image)
			
		
		if key == ord("q"):
			break
	cv2.destroyAllWindows()
		
tracking_thread = threading.Thread(target=object_tracker)
tracking_thread.start()

#-----------------------------------------------------------------------


# GPIO pin initialization function
def init():
	gpio.setmode(gpio.BOARD)
	gpio.setup(31, gpio.OUT)
	gpio.setup(33, gpio.OUT)
	gpio.setup(35, gpio.OUT)
	gpio.setup(37, gpio.OUT)
	gpio.setup(36,gpio.OUT) # gripper
	gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP)
	gpio.setup(7, gpio.IN, pull_up_down = gpio.PUD_UP)


# Function for turning off all pin outputs
def gameover():
	gpio.output(31, False)
	gpio.output(33, False)
	gpio.output(35, False)
	gpio.output(37, False)
	gpio.output(36, False)
	#gpio.cleanup()

def emergency_stop(signal_received, frame):
	print("Emergency stop")
	gameover()
	gpio.cleanup()
	sys.exit(0)

signal.signal(signal.SIGINT, emergency_stop)

# Initialize GPIO pins and pwms for both wheel sides and directions
init()
lf_pwm = gpio.PWM(31,50)
rf_pwm = gpio.PWM(37,50)
lb_pwm = gpio.PWM(33,50)
rb_pwm = gpio.PWM(35,50)
time.sleep(0.1)

init()
lf_pwm.start(0)
lb_pwm.start(0)
rf_pwm.start(0)
rb_pwm.start(0)



def turn_robot():
	while True:
		if object_detected:
			if rotate_target <= -2:
				duty = 50
				delay = abs(rotate_target)*0.05
				lf_pwm.ChangeDutyCycle(duty)
				rb_pwm.ChangeDutyCycle(duty)
				lb_pwm.ChangeDutyCycle(0)
				rf_pwm.ChangeDutyCycle(0)
				time.sleep(delay)
				lf_pwm.ChangeDutyCycle(0)
				rb_pwm.ChangeDutyCycle(0)
			elif rotate_target >= 2:
				duty = 50
				delay = abs(rotate_target)*0.05
				lb_pwm.ChangeDutyCycle(duty)
				rf_pwm.ChangeDutyCycle(duty)
				lf_pwm.ChangeDutyCycle(0)
				rb_pwm.ChangeDutyCycle(0)
				time.sleep(delay)
				lb_pwm.ChangeDutyCycle(0)
				rf_pwm.ChangeDutyCycle(0)
			else:
				lf_pwm.ChangeDutyCycle(0)
				lb_pwm.ChangeDutyCycle(0)
				rf_pwm.ChangeDutyCycle(0)
				rb_pwm.ChangeDutyCycle(0)
		time.sleep(0.5)


turn_robot()



			
		
rb_pwm.stop()
rf_pwm.stop()
lf_pwm.stop()
lb_pwm.stop()
gameover()
gpio.cleanup()
		
		
	


	
# def target_tracking():
	# rotate_target = 0
	# for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
			# #grab current frame
			
		# image = frame.array
		# image = cv2.flip(image, -1)
		# image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		# mask = cv2.inRange(image_hsv, lower_bound, upper_bound) 
		# contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		
		# if len(contours) > 0:
			# largest_contour = max(contours, key=cv2.contourArea)
			# rect = cv2.minAreaRect(largest_contour)
			# (x,y),box_dimensions,angle = rect
			# rotate_target = round((320-x)*0.061)
			# box = cv2.boxPoints(rect)
			# box = np.int0(box)
			# target_text = str(rotate_target)+" degrees"
			# cv2.drawContours(image, [box], 0, (0,0,0), 2)
	
		
		# if rotate_target < -2:
			# duty = 50
			# lf_pwm.ChangeDutyCycle(duty)
			# rb_pwm.ChangeDutyCycle(duty)
			# lb_pwm.ChangeDutyCycle(0)
			# rf_pwm.ChangeDutyCycle(0)
			# cv2.putText(image,target_text,(0,50),cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0),2)
	
		# elif rotate_target > 2:
			# duty = 50
			# lb_pwm.ChangeDutyCycle(duty)
			# rf_pwm.ChangeDutyCycle(duty)
			# lf_pwm.ChangeDutyCycle(0)
			# rb_pwm.ChangeDutyCycle(0)
			# cv2.putText(image,target_text,(0,50),cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0),2)
	
		# else:
			# lf_pwm.ChangeDutyCycle(0)
			# lb_pwm.ChangeDutyCycle(0)
			# rf_pwm.ChangeDutyCycle(0)
			# rb_pwm.ChangeDutyCycle(0)
			# cv2.putText(image,"Ready to retrieve",(0,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),2)
			
	
		# cv2.imshow("Frame", image)
		
		# key = cv2.waitKey(1) & 0xFF
		# rawCapture.truncate(0)
		
		# if key == ord("r") and not recording: #start recording
			# recording = True
			# rec_start = datetime.now()
			# rec_start_string = rec_start.strftime("%d-%m-%Y-%H_%M_%S")
			# out = cv2.VideoWriter(rec_start_string+'.avi', fourcc, 12, (640,480))
			# print("recording started at "+rec_start_string)
			
		# if key == ord("t") and recording: #stop recording
			# recording = False
			# out.release()
			# out = None
			# print("recording stopped")
		
		# if recording:
			# out.write(image)
		
		
		
		
		# if key == ord("q"):
			# break
	# cv2.destroyAllWindows()
	
	# rb_pwm.stop()
	# rf_pwm.stop()
	# lf_pwm.stop()
	# lb_pwm.stop()
	# gameover()
	# gpio.cleanup()

