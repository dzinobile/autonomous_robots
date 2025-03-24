#import modules
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

#initialize trig and echo pin values
trig = 16
echo = 18

#initialize video writer
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = None
recording = False

#time step for forward, backward, left, and right movements
move_step = 0.2

#initialize gpio pins as outputs or inputs
def init():
	gpio.setmode(gpio.BOARD)
	gpio.setup(31,gpio.OUT)
	gpio.setup(33,gpio.OUT)
	gpio.setup(35,gpio.OUT)
	gpio.setup(37,gpio.OUT)
	gpio.setup(36,gpio.OUT)
	gpio.setup(trig, gpio.OUT)
	gpio.setup(echo, gpio.IN)

#function for stopping car movement
def stop_car():
	gpio.output(31,False)
	gpio.output(33,False)
	gpio.output(35,False)
	gpio.output(37,False)

#move forward for time step amount
def forward():
	gpio.output(31,True)
	gpio.output(33,False)
	gpio.output(35,False)
	gpio.output(37,True)
	time.sleep(move_step)
	stop_car()

#move backward for time step amount
def backward():
	gpio.output(31,False)
	gpio.output(33,True)
	gpio.output(35,True)
	gpio.output(37,False)
	time.sleep(move_step)
	stop_car()

#turn right for time step amount
def pivot_right():
	gpio.output(31,True)
	gpio.output(33,False)
	gpio.output(35,True)
	gpio.output(37,False)
	time.sleep(move_step)
	stop_car()

#turn left for time step amount
def pivot_left():
	gpio.output(31,False)
	gpio.output(33,True)
	gpio.output(35,False)
	gpio.output(37,True)
	time.sleep(move_step)
	stop_car()

#measure distance with ultrasonic sensor
def distance():
		
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
	distance = str(round(distance, 2))
	print("Distance: "+distance+" mm")
	return distance

init() #initialize pins

#initialize gripper pwm signal
pwm_setting = 8 #set gripper to fully open
pwm = gpio.PWM(36, 50)
pwm.start(pwm_setting)

#initialize user commands and info printed on HUD
print("Press WASD keys to move car")
print("Press r to open / e to close gripper")
action = "---"
dist = distance()

#continuously grab camera image
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	
	#grab current frame
	image = frame.array
	image = cv2.flip(image, -1)
	

	key = cv2.waitKey(1) & 0xFF
	#movement commands
	if key == ord("w"):
		forward()
		dist = distance()
		action = "Moving forward"
	if key == ord("s"):
		backward()
		dist = distance()
		action = "Moving backward"
	if key == ord("a"):
		pivot_left()
		dist = distance()
		action = "Turning left"
	if key == ord("d"):
		pivot_right()
		dist = distance()
		action = "Turning right"

	#gripper commands
	if key == ord("r"):
		action = "Closing gripper"
		stop_car()
		if pwm_setting > 4:
			pwm_setting = pwm_setting-0.5
			pwm.ChangeDutyCycle(pwm_setting)
		else:
			print("gripper fully closed")
	if key == ord("e"):
		action = "Opening gripper"
		stop_car()
		if pwm_setting < 8:
			pwm_setting = pwm_setting+0.5
			pwm.ChangeDutyCycle(pwm_setting)

	#video recording commands
	if key == ord("x") and not recording: #start recording
		recording = True
		rec_start = datetime.now()
		rec_start_string = rec_start.strftime("%d-%m-%Y-%H_%M_%S")
		out = cv2.VideoWriter('teleoperation.avi', fourcc, 10, (640,480))
		print("recording started at "+rec_start_string)
	
	if key == ord("c") and recording: #stop recording
		recording = False
		out.release()
		out = None
		end_time_string = now.strftime("%d-%m-%Y-%H_%M_%S")
		print("recording stopped at "+end_time_string)

	#add HUD info to image
	cv2.rectangle(image, (20,2), (400,60),(255,255,255),-1)
	cv2.putText(image, action, (20,25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0),2)
	cv2.putText(image, "Distance: "+dist+" cm", (20,55), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0),2)
	cv2.imshow("Frame", image)
	
	#if recording, write image to video
	if recording:
		out.write(image)
		now = datetime.now()
		rec_time = (now-rec_start).total_seconds()
		print(f"Elapsed time: {rec_time} seconds")
	
	#press q to quit	
	if key == ord("q"):
		stop_car()
		break
	
	#clear the stream for next frame
	rawCapture.truncate(0)
	



cv2.destroyAllWindows() #close display
pwm.stop() #turn off pwm signal
gpio.cleanup() #clean up gpio pins
