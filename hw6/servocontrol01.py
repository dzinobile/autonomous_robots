# import modules
import RPi.GPIO as gpio
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
from datetime import datetime

#initialize camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 10
rawCapture = PiRGBArray(camera, size=(640,480))

#warmup
time.sleep(0.1)

#initialize font settings
org = (20, 25)
fontFace = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 1
color = (0, 0, 0)
thickness = 2

#initialize video writer
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('servocontrol01.avi', fourcc, 1, (640,480)) #framerate set to 1 frame per second

#initialize gpio output for servo
gpio.setmode(gpio.BOARD)
gpio.setup(36,gpio.OUT)

#initialize pwm settings
pwm_setting = 8
pwm = gpio.PWM(36, 50)
pwm.start(pwm_setting)
duty_cycle_string = "Duty: "+str(pwm_setting)+"%"
print("Press r to close or e to open grippers")


for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	
	#grab current frame
	image = frame.array
	image = cv2.flip(image, -1)
	
	#display the frame
	cv2.putText(image, duty_cycle_string, org, fontFace, fontScale, color, thickness)
	cv2.imshow("Frame", image)
	key = cv2.waitKey(1) & 0xFF
	
	#closing gripper
	if key == ord("r"):
		if pwm_setting > 4: #lower bound for fully closed gripper
			pwm_setting = pwm_setting-0.5
			pwm.ChangeDutyCycle(pwm_setting)
			duty_cycle_string = "Closing gripper, Duty: "+str(pwm_setting)+"%"
		else:
			duty_cycle_string = "Gripper fully closed, Duty: "+str(pwm_setting)+"%"
			print("gripper fully closed")
			
		time.sleep(2) #give additional time for gripper to move
		rawCapture.truncate(0) #clear image buffer
		camera.capture(rawCapture, format="bgr") #capture image
		d_img = rawCapture.array
		d_img = cv2.flip(d_img, -1)
		cv2.putText(d_img, duty_cycle_string, org, fontFace, fontScale, color, thickness)
		out.write(d_img) #write image to video
		print("screenshot captured")

	#opening gripper	
	if key == ord("e"):
		if pwm_setting < 8: #upper bound for fully opened gripper
			pwm_setting = pwm_setting+0.5
			pwm.ChangeDutyCycle(pwm_setting)
			duty_cycle_string = "Opening gripper, Duty: "+str(pwm_setting)+"%"
		else:
			duty_cycle_string = "Gripper fully opened, Duty: "+str(pwm_setting)+"%"
			print("gripper fully opened")
			
		time.sleep(2) #give additional time for gripper to move
		rawCapture.truncate(0) #clear image buffer
		camera.capture(rawCapture, format="bgr") #capture image
		d_img = rawCapture.array
		d_img = cv2.flip(d_img, -1)
		cv2.putText(d_img, duty_cycle_string, org, fontFace, fontScale, color, thickness)
		out.write(d_img) #write image to video
		print("screenshot captured")
	
	# press q to quit
	if key == ord("q"):
		break
	
	
	#clear the stream for next frame
	rawCapture.truncate(0)
	


out.release() #release video
cv2.destroyAllWindows() #close display
pwm.stop() #turn off pwm signal
gpio.cleanup() #clean up gpio pins
