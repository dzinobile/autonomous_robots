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
import smtplib
from smtplib import SMTP
from smtplib import SMTPException
import email
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage

smtpUser = 'enpm701zinobile@gmail.com'
smtpPass = 'caafbauozztsduvz'
toAdd = ['enpm701zinobile@gmail.com','ENPM701S19@gmail.com']
fromAdd = smtpUser




# IMU ------------------------------------------------------------------
ser = serial.Serial('/dev/ttyUSB0', 9600)
current_angle_imu = 0
start_angle_imu = 0
IMU_ready = False

def read_serial_data():
	ser_count = 0 # Initial serial count
	global current_angle_imu # Current angle reading
	global IMU_ready # Boolean to prevent any movement until first 10 lines are read
	
	# Continuously read data
	while True:
		ser.reset_input_buffer() # Reset buffer to ignore old readings
		line = ser.readline() # Wait for and read the next new line 
		ser_count +=1
		
		if ser_count > 10:
			
			# Strip unneccessary info
			line = line.rstrip().lstrip()
			line = str(line)
			line = line.strip("'")
			line = line.strip("b'")
			line = line.strip("X: ")
			
			current_angle_imu = float(line) # Update current angle global variable
			#print(current_angle_imu)
			IMU_ready = True # First 10 lines have been read
			
serial_thread = threading.Thread(target=read_serial_data, daemon=True)
serial_thread.start()

# Pause until first 10 serial readings are taken
print("Starting IMU...")
while True:
	if IMU_ready:
		print("Complete")
		break
#-----------------------------------------------------------------------
			



#TRACKER ---------------------------------------------------------------
object_detected = False
x = 320
y = 240
rotate_target = 0
ready_to_retrieve = False
retrieve_command = False
current_action = "None"
def object_tracker():
	global current_action
	global object_detected
	global rotate_target
	global x
	global y
	global ready_to_retrieve
	global retrieve_command
	
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
			cv2.rectangle(image,(0,0),(640,75),(255,255,255),-1)
		else:
			object_detected = False
		if current_action == "turning to object":
			if -2 < rotate_target < 2:
				cv2.putText(image,"Ready to retrieve",(0,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),2)
				ready_to_retrieve = True
				
			else:
				ready_to_retrieve = False
				target_text = "Rotating "+str(rotate_target)+" degrees"
				cv2.putText(image,target_text,(0,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),2)
		elif current_action == "navigating to object":
			cv2.putText(image,"Navigating to object",(0,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),2)
		elif current_action == "closing gripper":
			cv2.putText(image,"Closing gripper",(0,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),2)
		elif current_action == "returning to start":
			cv2.putText(image,"Returning to start",(0,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),2)
		elif current_action == "turning around":
			cv2.putText(image,"Turning around",(0,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),2)
		elif current_action == "opening gripper":
			cv2.putText(image,"Opening gripper",(0,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),2)
		elif current_action == "sending email":
			cv2.putText(image,"Sending email",(0,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),2)
		elif current_action == "inching up to object":
			cv2.putText(image,"Inching up to object",(0,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),2)
		
		cv2.imshow("Frame", image)
		key = cv2.waitKey(1) & 0xFF
		rawCapture.truncate(0)
			
		
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
g_pwm = gpio.PWM(36,50)
time.sleep(0.1)


lf_pwm.start(0)
lb_pwm.start(0)
rf_pwm.start(0)
rb_pwm.start(0)
g_pwm.start(8)

def inch_up(y_target,duty):
	travel = 0
	left_counter = np.uint64(0)
	left_button = int(0)
	right_counter = np.uint64(0)
	right_button = int(0)

	while y < y_target:
		if int(gpio.input(7)) != int(left_button):
			left_button = int(gpio.input(7))
			left_counter += 1
		if int(gpio.input(12)) != int(right_button):
			right_button = int(gpio.input(12))
			right_counter += 1
		left_travel =  (left_counter/20)*(3.1415*0.065)
		right_travel = (right_counter/20)*(3.1415*0.065)
		travel = (left_travel+right_travel)/2
		lf_pwm.ChangeDutyCycle(duty)
		rf_pwm.ChangeDutyCycle(duty)
		time.sleep(0.1)
		lf_pwm.ChangeDutyCycle(0)
		rf_pwm.ChangeDutyCycle(0)
		time.sleep(0.3)
		
	lf_pwm.ChangeDutyCycle(0)
	rf_pwm.ChangeDutyCycle(0)
	return travel


def move_to_target(y_target,duty):
	travel = 0
	start_angle = current_angle_imu
	
	left_counter = np.uint64(0)
	left_button = int(0)
	right_counter = np.uint64(0)
	right_button = int(0)
	
	Kp = 1
	integral = 0
	previous_error = 0
	init()
	left_duty = duty
	right_duty = duty
	lf_pwm.start(left_duty)
	rf_pwm.start(right_duty)


	while y < y_target:
		error = current_angle_imu - start_angle
		if error > 180:
			error -= 360
		elif error < -180:
			error += 360

		# Update tick count if status changes
		if int(gpio.input(7)) != int(left_button):
			left_button = int(gpio.input(7))
			left_counter += 1
		if int(gpio.input(12)) != int(right_button):
			right_button = int(gpio.input(12))
			right_counter += 1

	
		correction = Kp*error
		
		left_duty -= correction
		right_duty += correction
		
		
		left_duty = max(min(left_duty, 100),10)
		right_duty = max(min(right_duty, 100),10)
		
		lf_pwm.ChangeDutyCycle(left_duty)
		rf_pwm.ChangeDutyCycle(right_duty)
		
		left_travel =  (left_counter/20)*(3.1415*0.065)
		right_travel = (right_counter/20)*(3.1415*0.065)
		travel = (left_travel+right_travel)/2
		
	lf_pwm.ChangeDutyCycle(0)
	lb_pwm.ChangeDutyCycle(0)
	rf_pwm.ChangeDutyCycle(0)
	rb_pwm.ChangeDutyCycle(0)
	return travel

def turn_robot_tracker():
	duty = 50
	i = 0
	while True:
		if object_detected:
			delay = abs(rotate_target)*0.05
			if rotate_target <= -2:
				lf_pwm.ChangeDutyCycle(duty)
				rb_pwm.ChangeDutyCycle(duty)
				lb_pwm.ChangeDutyCycle(0)
				rf_pwm.ChangeDutyCycle(0)
				time.sleep(delay)
				lf_pwm.ChangeDutyCycle(0)
				rb_pwm.ChangeDutyCycle(0)
				i = 0
			elif rotate_target >= 2:
				lb_pwm.ChangeDutyCycle(duty)
				rf_pwm.ChangeDutyCycle(duty)
				lf_pwm.ChangeDutyCycle(0)
				rb_pwm.ChangeDutyCycle(0)
				time.sleep(delay)
				lb_pwm.ChangeDutyCycle(0)
				rf_pwm.ChangeDutyCycle(0)
				i = 0
			else:
				lf_pwm.ChangeDutyCycle(0)
				lb_pwm.ChangeDutyCycle(0)
				rf_pwm.ChangeDutyCycle(0)
				rb_pwm.ChangeDutyCycle(0)
				i += 1
			if i > 5:
				break
		time.sleep(0.3)
	return current_angle_imu
		
		
def turn_around():
	duty = 80
	goal_angle = current_angle_imu - 180
	if goal_angle > 180:
		goal_angle -= 360
	elif goal_angle < -180:
		goal_angle += 360
	
	while True:
		turn_target = current_angle_imu - goal_angle
		if turn_target > 180:
			turn_target -= 360
		elif turn_target < -180:
			turn_target += 360
		delay = abs(turn_target)*0.01
		delay = max(delay,0.1)
		if turn_target <= -2:
			lf_pwm.ChangeDutyCycle(duty)
			rb_pwm.ChangeDutyCycle(duty)
			lb_pwm.ChangeDutyCycle(0)
			rf_pwm.ChangeDutyCycle(0)
			time.sleep(delay)
			lf_pwm.ChangeDutyCycle(0)
			rb_pwm.ChangeDutyCycle(0)
			i = 0
		elif turn_target >= 2:
			lb_pwm.ChangeDutyCycle(duty)
			rf_pwm.ChangeDutyCycle(duty)
			lf_pwm.ChangeDutyCycle(0)
			rb_pwm.ChangeDutyCycle(0)
			time.sleep(delay)
			lb_pwm.ChangeDutyCycle(0)
			rf_pwm.ChangeDutyCycle(0)
			i = 0
		else:
			lf_pwm.ChangeDutyCycle(0)
			lb_pwm.ChangeDutyCycle(0)
			rf_pwm.ChangeDutyCycle(0)
			rb_pwm.ChangeDutyCycle(0)
			i += 1
		if i > 5:
			break
	time.sleep(0.3)
		
def back_up(distance,heading):
	duty = 50
	travel = 0
	
	
	left_counter = np.uint64(0)
	left_button = int(0)
	right_counter = np.uint64(0)
	right_button = int(0)
	
	Kp = 1
	integral = 0
	previous_error = 0
	left_duty = duty
	right_duty = duty
	lb_pwm.ChangeDutyCycle(left_duty)
	rb_pwm.ChangeDutyCycle(right_duty)


	while travel < distance:
		error = current_angle_imu - heading
		if error > 180:
			error -= 360
		elif error < -180:
			error += 360

		# Update tick count if status changes
		if int(gpio.input(7)) != int(left_button):
			left_button = int(gpio.input(7))
			left_counter += 1
		if int(gpio.input(12)) != int(right_button):
			right_button = int(gpio.input(12))
			right_counter += 1

	
		correction = Kp*error
		
		left_duty += correction
		right_duty -= correction
		
		
		left_duty = max(min(left_duty, 100),10)
		right_duty = max(min(right_duty, 100),10)
		
		lb_pwm.ChangeDutyCycle(left_duty)
		rb_pwm.ChangeDutyCycle(right_duty)
		
		left_travel =  (left_counter/20)*(3.1415*0.065)
		right_travel = (right_counter/20)*(3.1415*0.065)
		travel = (left_travel+right_travel)/2
		
	lf_pwm.ChangeDutyCycle(0)
	lb_pwm.ChangeDutyCycle(0)
	rf_pwm.ChangeDutyCycle(0)
	rb_pwm.ChangeDutyCycle(0)
	
def send_email():
	subject = 'zinobile hw9' 
	msg = MIMEMultipart()
	msg['Subject'] = subject
	msg['From'] = fromAdd
	msg['To'] = ",".join(toAdd)
	msg.preamble = "hw9"
	body = MIMEText("Object retrieved. Daniel Zinobile hw9 complete.")
	msg.attach(body)
	s = smtplib.SMTP('smtp.gmail.com', 587)
	s.ehlo()
	s.starttls()
	s.ehlo()
	
	s.login(smtpUser,smtpPass)
	s.sendmail(fromAdd, toAdd, msg.as_string())
	s.quit()
	print("email delivered")

		

current_action = "turning to object"
initial_heading = turn_robot_tracker()

current_action = "navigating to object"
travel_distance = move_to_target(350,50)	

current_action = "turning to object"
turn_robot_tracker()

current_action = "inching up to object"
travel_distance += inch_up(420,50)


current_action = "closing gripper"
g_pwm.ChangeDutyCycle(4)
time.sleep(2)
g_pwm.ChangeDutyCycle(8)
time.sleep(2)
g_pwm.ChangeDutyCycle(4)
time.sleep(2)

current_action = "returning to start"
back_up(travel_distance,initial_heading)

current_action = "turning around"
turn_around()

current_action = "opening gripper"
g_pwm.ChangeDutyCycle(8)
time.sleep(3)

current_action = "sending email"
send_email()







			
		
rb_pwm.stop()
rf_pwm.stop()
lf_pwm.stop()
lb_pwm.stop()
gameover()
gpio.cleanup()
		
		
	


	


