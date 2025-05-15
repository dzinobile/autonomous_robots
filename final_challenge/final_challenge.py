
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
import time
import serial
import threading
import heapq
import pigpio
import signal
import sys
from matplotlib import pyplot as plt
from multiprocessing import Process
import imaplib
import email
import time
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage
from datetime import datetime

                
#IMU STARTUP -----------------------------------------------------------
robot_position = [0,0,0]
ser = serial.Serial('/dev/ttyUSB0', 9600)
IMU_ready = False
def read_serial_data():
	print("read_serial_data")
	ser_count = 0 # Initial serial count
	global robot_position
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
			
			robot_position[2] = (((360-round(float(line)))%360)+180)%360 # Update current angle global variable
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

#GPIO STARTUP-----------------------------------------------------------

# Initialize pin reference names
pi = pigpio.pi()
trig = 23 # sonar trigger
echo = 24 # sonar listener
lf = 6 # left forwards
lb = 13 # left backwards
rb = 19 # right backwards
rf = 26 # right forwards
gr = 16 # gripper
re = 18 # right encoder
le = 4 # left encoder

pi.set_mode(lf, pigpio.OUTPUT) 
pi.set_mode(lb, pigpio.OUTPUT)
pi.set_mode(rb, pigpio.OUTPUT)
pi.set_mode(rf, pigpio.OUTPUT)
pi.set_mode(gr, pigpio.OUTPUT)
pi.set_mode(trig, pigpio.OUTPUT)
pi.set_mode(echo, pigpio.INPUT)
pi.set_mode(re, pigpio.INPUT)
pi.set_pull_up_down(re, pigpio.PUD_UP)
pi.set_mode(le, pigpio.INPUT)  
pi.set_pull_up_down(le, pigpio.PUD_UP)

pi.set_PWM_frequency(lf,50)
pi.set_PWM_frequency(lb,50)
pi.set_PWM_frequency(rb,50)
pi.set_PWM_frequency(rf,50)
pi.set_PWM_frequency(gr,50)

pi.set_PWM_dutycycle(lf,0)
pi.set_PWM_dutycycle(lb,0)
pi.set_PWM_dutycycle(rb,0)
pi.set_PWM_dutycycle(rf,0)
pi.set_PWM_dutycycle(gr,int(round(0.085*255))) # Set gripper to open



def gameover():
	pi.write(lf,0)
	pi.write(lb,0)
	pi.write(rb,0)
	pi.write(rf,0)
	pi.write(gr,0)
	pi.write(trig,0)
	pi.stop()

def emergency_stop(signal_received, frame):
	print("Emergency stop")
	gameover()
	sys.exit(0)

signal.signal(signal.SIGINT, emergency_stop)

#-----------------------------------------------------------------------





#BLOCK POSITIONS TRACKER STARTUP----------------------------------------
rotate_target_closest_blue = 0 # Angle away from center for closest blue block
pixel_y_closest_blue = 0 # Pixel y position for closest blue block
closest_dist_blue = 0 # Distance from closest blue block

rotate_target_closest_green = 0
pixel_y_closest_green = 0
closest_dist_green = 0

rotate_target_closest_red = 0
pixel_y_closest_red = 0
closest_dist_red = 0

# Indicators for blocks in robot straight line path
blue_inpath = False
green_inpath = False
red_inpath = False

# Control for masking out bottom of frame when block gripped
block_gripped = False

sendemail = False

def euc_distance(p1,p2):
	p1x,p1y = p1[:2]
	p2x,p2y = p2[:2]
	dist = np.sqrt((p1x-p2x)**2 + (p1y-p2y)**2)
	return dist

def send_email(image):
	smtpUser = 'enpm701zinobile@gmail.com'
	smtpPass = 'caafbauozztsduvz'
	toAdd = ['enpm701zinobile@gmail.com','enpm809ts19@gmail.com']
	fromAdd = smtpUser
	pic_time = datetime.now().strftime('%Y%m%d%H%M%S')
	picture = cv2.imwrite(pic_time+".jpg",image)
	subject = 'image recorded at' + pic_time
	msg = MIMEMultipart()
	msg['Subject'] = subject
	msg['From'] = fromAdd
	msg['To'] = ",".join(toAdd)
	msg.preamble = "Image recorded at " + pic_time
	body = MIMEText("Image recorded at " + pic_time)
	msg.attach(body)
	fp = open(pic_time+'.jpg','rb')
	img = MIMEImage(fp.read())
	fp.close()
	msg.attach(img)
	s = smtplib.SMTP('smtp.gmail.com', 587)
	s.ehlo()
	s.starttls()
	s.ehlo()
	
	s.login(smtpUser,smtpPass)
	s.sendmail(fromAdd, toAdd, msg.as_string())
	s.quit()


# def average(p1,p2):
	# p1x,p1y = p1
	# p2x,p2y = p2
	# x = (p1x+p2x)/2
	# y = (p1y+p2y)/2
	# return (x,y)

# Function for whether or not block is in robot straight line path
def inpath(box):
    for point in box:
        x = point[0]
        y = point[1]
        if y > 245 and y > 0.734 * x + 9.728 and y > -0.734 * x + 480:
            return True
    return False

def object_tracker():
	
	# Global variables for continuous updating
	global rotate_target_closest_blue
	global rotate_target_closest_green
	global rotate_target_closest_red
	global pixel_y_closest_blue
	global pixel_y_closest_green
	global pixel_y_closest_red
	global closest_dist_blue
	global closest_dist_green
	global closest_dist_red
	global blue_inpath
	global green_inpath
	global red_inpath
	global sendemail
	
	camera = PiCamera()
	camera.resolution = (640, 480)
	camera.framerate = 24
	rawCapture = PiRGBArray(camera, size=(640,480))
	time.sleep(2)
	gains = camera.awb_gains
	camera.awb_mode = 'off'
	camera.awb_gains = gains
	print("camera set")
	
	# ROOM
	# blue_lower = np.array([101,50,51])
	# blue_upper = np.array([134,255,255])
	# green_lower = np.array([23,25,0])
	# green_upper = np.array([42,255,255])
	# red_lower1 = np.array([170,120,75])
	# red_upper1 = np.array([255,255,255])
	# red_lower2 = np.array([0, 120, 75])
	# red_upper2 = np.array([10, 255, 255])
	
	#FINAL CHALLENGE
	blue_lower = np.array([68,0,10]) 
	blue_upper = np.array([130,255,255])
	green_lower = np.array([39,0,10])
	green_upper = np.array([92,255,255])
	red_lower1 = np.array([170,120,100])
	red_upper1 = np.array([255,255,255])
	red_lower2 = np.array([5, 120, 100])
	red_upper2 = np.array([10, 255, 255])
			
	
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		
		image = frame.array
		image = cv2.flip(image, -1)
		image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		image_hsv[0:220,:] = (0,0,0) # Mask out top half for walls 

		# Mask out tips of open gripper
		image_hsv = cv2.rectangle(image_hsv,(0,445),(160,480),(0,0,0),-1)
		image_hsv = cv2.rectangle(image_hsv,(480,445),(640,480),(0,0,0),-1)
		
		if block_gripped:
			image_hsv[375:480,:] = (0,0,0) # Mask bottom if block in grippers
			
		blue_mask = image_hsv.copy()
		green_mask = image_hsv.copy()
		red_mask = image_hsv.copy()

		blue_mask = cv2.inRange(blue_mask, blue_lower, blue_upper)
		green_mask = cv2.inRange(green_mask, green_lower, green_upper)
		red_mask1 = cv2.inRange(red_mask, red_lower1, red_upper1)
		red_mask2 = cv2.inRange(red_mask, red_lower2, red_upper2)
		red_mask = cv2.bitwise_or(red_mask1,red_mask2)
		
		blue_cont, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		green_cont,_ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		red_cont, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		
		# Filter out unusually small or large contours
		min_area = 20
		max_area = 30000
		blue_contours = [cnt for cnt in blue_cont if cv2.contourArea(cnt) > min_area]
		green_contours = [cnt for cnt in green_cont if cv2.contourArea(cnt) > min_area]
		red_contours = [cnt for cnt in red_cont if cv2.contourArea(cnt) > min_area]
		
		blue_contours = [cnt for cnt in blue_contours if cv2.contourArea(cnt) < max_area]
		green_contours = [cnt for cnt in green_contours if cv2.contourArea(cnt) < max_area]
		red_contours = [cnt for cnt in red_contours if cv2.contourArea(cnt) < max_area]
		
		# Get largest 3 contours only
		blue_contours = sorted(blue_contours, key=cv2.contourArea, reverse=True)[:(min(3,len(blue_contours)))]
		green_contours = sorted(green_contours,key=cv2.contourArea, reverse=True)[:(min(3,len(green_contours)))]
		red_contours = sorted(red_contours,key=cv2.contourArea, reverse=True)[:(min(3,len(red_contours)))]

		blue_inpath = False
		green_inpath = False
		red_inpath = False


		if len(blue_contours) > 0:
			closest_dist_blue = 0
			for blue_contour in blue_contours:
				rect = cv2.minAreaRect(blue_contour)
				(rect_x,rect_y),(rect_w,rect_h),rect_a = rect
				if rect_w>rect_h: # minAreaRect default is width > height
					rect_w, rect_h = rect_h, rect_w
				if 0.5 < (rect_h/rect_w) < 3: # filter out anything with extreme aspect ratio
					rotate_target = round((320-rect_x)*0.061)
					distance_blue = 1654.5*(rect_h**-0.798) # Distance formula based on height
					if closest_dist_blue==0 or distance_blue < closest_dist_blue: # Get distance of closest block
						rotate_target_closest_blue = rotate_target
						pixel_y_closest_blue = rect_y
						closest_dist_blue = distance_blue
					box = cv2.boxPoints(rect)
					box = np.int0(box)
					if inpath(box):
						blue_inpath = True
					cv2.drawContours(image, [box], 0, (255,0,0), 1)
					cv2.putText(image,str(int(distance_blue)),(int(rect_x+rect_w/2),int(rect_y+rect_h/2)),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),1)
				
				
		if len(green_contours) > 0:
			closest_dist_green = 0
			for green_contour in green_contours:
				rect = cv2.minAreaRect(green_contour)
				(rect_x,rect_y),(rect_w,rect_h),rect_a = rect
				if rect_w>rect_h:
					rect_w, rect_h = rect_h, rect_w
				if 0.5 < (rect_h/rect_w) < 3 :
					rotate_target = round((320-rect_x)*0.061)
					distance_green = 1613.9*(rect_h**-0.783)
					if closest_dist_green ==0 or distance_green < closest_dist_green:
						rotate_target_closest_green = rotate_target
						pixel_y_closest_green = rect_y
						closest_dist_green = distance_green
					box = cv2.boxPoints(rect)
					box = np.int0(box)
					if inpath(box):
						green_inpath = True
					cv2.drawContours(image, [box], 0, (0,255,0), 1)
					cv2.putText(image,str(int(distance_green)),(int(rect_x+rect_w/2),int(rect_y+rect_h/2)),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),1)
				
		if len(red_contours) > 0:
			closest_dist_red = 0
			for red_contour in red_contours:
				rect = cv2.minAreaRect(red_contour)
				(rect_x,rect_y),(rect_w,rect_h),rect_a = rect
				if rect_w>rect_h:
					rect_w, rect_h = rect_h, rect_w
				if 0.5 < (rect_h/rect_w) < 3 :
					distance_red = 1654.5*(rect_h**-0.798)
					rotate_target = round((320-rect_x)*0.061)
					if closest_dist_red==0 or distance_red < closest_dist_red:
						rotate_target_closest_red = rotate_target
						pixel_y_closest_red = rect_y
						closest_dist_red = distance_red
					box = cv2.boxPoints(rect)
					box = np.int0(box)
					if inpath(box):
						red_inpath = True
					cv2.drawContours(image, [box], 0, (0,0,255), 1)
					cv2.putText(image,str(int(distance_red)),(int(rect_x+rect_w/2),int(rect_y+rect_h/2)),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),1)
		
		# Display robot straight line path
		cv2.line(image,(0,480),(320,245),(255,255,255),1) 
		cv2.line(image,(640,480),(320,245),(255,255,255),1)
		
		
		cv2.putText(image,"Robot position: "+str(robot_position),(10,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),1)
		
		# Send email of frame when prompted
		if sendemail:
			sendemail = False
			send_email(image)
		
		
		
		cv2.imshow("Frame", image)
		key = cv2.waitKey(1) & 0xFF
		rawCapture.truncate(0)
		
		if key == ord("q"):
			break
	cv2.destroyAllWindows()

# Create thread for block tracker
tracking_thread = threading.Thread(target=object_tracker)
tracking_thread.start()
#-----------------------------------------------------------------------


#MOVEMENT COMMANDS------------------------------------------------------

# Turning required to move in steps to overcome friction of floor
def turn_to_angle(goal_angle):
	print("turning to " + str(round(goal_angle)))
	start_angle = robot_position[2]
	turn_angle = (goal_angle-start_angle+180)%360-180
	duty = int(round(255))
	i = 0
	while True:
		error = (goal_angle-robot_position[2]+180)%360-180
		delay = (abs(error)*0.02)
		if delay > 0.5:
			delay = 0.5
		
		if error > 2:
			pi.set_PWM_dutycycle(lf,0)
			pi.set_PWM_dutycycle(rb,0)
			pi.set_PWM_dutycycle(lb,duty)
			pi.set_PWM_dutycycle(rf,duty)
			time.sleep(delay)
			pi.set_PWM_dutycycle(rf,0)
			pi.set_PWM_dutycycle(lb,0)
			i = 0
		elif error < -2:
			pi.set_PWM_dutycycle(lb,0)
			pi.set_PWM_dutycycle(rf,0)
			pi.set_PWM_dutycycle(lf,duty)
			pi.set_PWM_dutycycle(rb,duty)
			time.sleep(delay)
			pi.set_PWM_dutycycle(lf,0)
			pi.set_PWM_dutycycle(rb,0)
			i = 0
		else:
			pi.set_PWM_dutycycle(lf,0)
			pi.set_PWM_dutycycle(rb,0)
			pi.set_PWM_dutycycle(lb,0)
			pi.set_PWM_dutycycle(rf,0)
			i+=1
		if i > 2:
			break
		time.sleep(0.3)
	pi.set_PWM_dutycycle(rf,0)
	pi.set_PWM_dutycycle(rb,0)
	pi.set_PWM_dutycycle(lf,0)
	pi.set_PWM_dutycycle(lb,0)
	
	

	
def move_forward_by_distance(inpt_dist):
	print("moving forward "+str(inpt_dist)+" cm")
	start_angle = robot_position[2] # start angle for IMU control loop

	# Initial state of encoders and travel distance
	travel = 0
	left_counter = np.uint64(0)
	left_button = int(0)
	right_counter = np.uint64(0)
	right_button = int(0)

	# PID loop using IMU data
	Kp = 5
	#Ki = 0.1
	#Kd = 0.1
	#integral = 0
	#previous_error = 0
	left_duty = int(round(255))
	right_duty = int(round(255))
	pi.set_PWM_dutycycle(lf,left_duty)
	pi.set_PWM_dutycycle(rf,right_duty)

	while travel < inpt_dist:
		error = robot_position[2] - start_angle
		if error > 180:
			error -= 360
		elif error < -180:
			error += 360
			
		#integral += error
		#derivative = error - previous_error
		
		
		if int(pi.read(le)) != int(left_button):
			left_button = int(pi.read(le))
			left_counter += 1
		if int(pi.read(re)) != int(right_button):
			right_button = int(pi.read(re))
			right_counter += 1


		correction = int(round(Kp*error)) #+ Ki*integral + Kd*derivative))
		#previous_error = error
		
		left_duty += correction
		right_duty -= correction
		
		
		left_duty = max(min(left_duty, 255),30)
		right_duty = max(min(right_duty, 255),30)
		
		pi.set_PWM_dutycycle(lf,left_duty)
		pi.set_PWM_dutycycle(rf,right_duty)
		
		left_travel =  (left_counter/20)*(3.1415*6.5)
		right_travel = (right_counter/20)*(3.1415*6.5)
		travel = (left_travel+right_travel)/2
	robot_position[0] = int(round(robot_position[0]+(travel*np.cos(np.deg2rad(start_angle)))))
	robot_position[1] = int(round(robot_position[1]+(travel*np.sin(np.deg2rad(start_angle)))))
	pi.set_PWM_dutycycle(lf,0)
	pi.set_PWM_dutycycle(rf,0)
	pi.set_PWM_dutycycle(lb,0)
	pi.set_PWM_dutycycle(rb,0)	

def move_backward_by_distance(inpt_dist):
	print("moving backwards "+str(inpt_dist)+" cm")

	start_angle = robot_position[2]
	travel = 0
	left_counter = np.uint64(0)
	left_button = int(0)
	right_counter = np.uint64(0)
	right_button = int(0)

	Kp = 5
	integral = 0
	previous_error = 0
	left_duty = int(round(255))
	right_duty = int(round(255))
	pi.set_PWM_dutycycle(lb,left_duty)
	pi.set_PWM_dutycycle(rb,right_duty)
	while travel < inpt_dist:
		error = robot_position[2] - start_angle
		if error > 180:
			error -= 360
		elif error < -180:
			error += 360
			
		# Update tick count if status changes
		if int(pi.read(le)) != int(left_button):
			left_button = int(pi.read(le))
			left_counter += 1
		if int(pi.read(re)) != int(right_button):
			right_button = int(pi.read(re))
			right_counter += 1

		correction = int(round(Kp*error))
		
		left_duty -= correction
		right_duty += correction
		
		left_duty = max(min(left_duty, 255),30)
		right_duty = max(min(right_duty, 255),30)
		
		pi.set_PWM_dutycycle(lb,left_duty)
		pi.set_PWM_dutycycle(rb,right_duty)
		
		left_travel =  (left_counter/20)*(3.1415*6.5)
		right_travel = (right_counter/20)*(3.1415*6.5)
		travel = (left_travel+right_travel)/2
	robot_position[0] = int(round(robot_position[0]-(travel*np.cos(np.deg2rad(start_angle)))))
	robot_position[1] = int(round(robot_position[1]-(travel*np.sin(np.deg2rad(start_angle)))))
	pi.set_PWM_dutycycle(lf,0)
	pi.set_PWM_dutycycle(rf,0)
	pi.set_PWM_dutycycle(lb,0)
	pi.set_PWM_dutycycle(rb,0)

# Turn to lower left corner of construction zone 
# in preparation to find clear path to construction zone
def turn_to_const():
	print("turning to construction home")
	(x1,y1) = robot_position[0],robot_position[1]
	(x2,y2) = (50,200)
	current_angle = robot_position[2]
	x3 = x2-x1
	y3 = y2-y1
	angle_to_reach = (np.degrees(np.arctan2(y3,x3)))%360
	turn_to_angle(angle_to_reach)
	

# Move to given xy coordinates
def move_forward_to_point(point_xy):
	print("moving to "+str(point_xy))
	(x1,y1) = robot_position[0],robot_position[1]
	(x2,y2) = point_xy
	current_angle = robot_position[2]
	x3 = x2-x1
	y3 = y2-y1
	angle_to_reach = (np.degrees(np.arctan2(y3,x3)))%360
	distance_to_move = int(round(euc_distance((x1,y1),point_xy)))
	turn_to_angle(angle_to_reach)
	move_forward_by_distance(distance_to_move)


# Function to turn right until unobstructed path to goal found
def turn_until_clear_path(cycle):
	print("turning until clear path")
	if cycle == None:
		color = None
	else:
		color = cycle%3
	duty = int(round(1*255))
	delay = 0.1
	start_angle = robot_position[2]
	angle_turned = ((start_angle - robot_position[2])%360)
	
	i = 0
	while i < 5 and angle_turned < 225: # Stop searching for paths if none found within range
		angle_turned = ((start_angle - robot_position[2])%360)
		
		# Conditions for what colors are permitted in robot path
		if color == 2:
			if blue_inpath and not green_inpath and not red_inpath:
				end_cond = True
			else:
				end_cond = False
		elif color == 1:
			if green_inpath and not blue_inpath and not red_inpath:
				end_cond = True
			else:
				end_cond = False
		elif color == 0:
			if red_inpath and not blue_inpath and not green_inpath:
				end_cond = True
			else:
				end_cond = False
		
		elif color == None:
			if not red_inpath and not blue_inpath and not green_inpath:
				end_cond = True
			else:
				end_cond = False
		if end_cond:
			pi.set_PWM_dutycycle(lb,0)
			pi.set_PWM_dutycycle(rf,0)
			pi.set_PWM_dutycycle(lf,0)
			pi.set_PWM_dutycycle(rb,0)
			i += 1
		else:
			pi.set_PWM_dutycycle(lf,duty)
			pi.set_PWM_dutycycle(rb,duty)
			pi.set_PWM_dutycycle(lb,0)
			pi.set_PWM_dutycycle(rf,0)
			time.sleep(delay)
			pi.set_PWM_dutycycle(lf,0)
			pi.set_PWM_dutycycle(rb,0)
			i = 0
		time.sleep(0.3)
	if i == 5: # wait 5 cycles to ensure robot has settled into a viable path with no backlash
		return True
	else:
		return False

# If no unobstructed path found, move 20 cm away down and to the right and try again 
def find_path(cycle):
	print("finding path")
	while True:
		if turn_until_clear_path(cycle):
			break
		turn_to_angle(315)
		turn_until_clear_path(None)
		move_forward_by_distance(20)
		turn_to_angle(45)
		
# Approach and pick up block from a distance
def pick_up_block(cycle):
	print("picking up block")
	duty = int(round(0.6*255))
	color = cycle%3
	delay = 0.1
	
	
	left_counter = np.uint64(0)
	left_button = int(0)
	right_counter = np.uint64(0)
	right_button = int(0)
	travel = 0
	i = 0
	n = 0
	creep = False
	while True:
		
		if color == 2:
			rot_targ = rotate_target_closest_blue
			pix_y = pixel_y_closest_blue
			dist_targ = closest_dist_blue
		elif color == 1:
			rot_targ = rotate_target_closest_green
			pix_y = pixel_y_closest_green
			dist_targ = closest_dist_green
		elif color == 0:
			rot_targ = rotate_target_closest_red
			pix_y = pixel_y_closest_red
			dist_targ = closest_dist_red
		if dist_targ < 50:
			creep = True
		
		if rot_targ <= -2:
			
			pi.set_PWM_dutycycle(lf,duty)
			pi.set_PWM_dutycycle(rb,duty)
			pi.set_PWM_dutycycle(lb,0)
			pi.set_PWM_dutycycle(rf,0)
			time.sleep(delay)
			pi.set_PWM_dutycycle(lf,0)
			pi.set_PWM_dutycycle(rb,0)
			

		elif rot_targ >= 2:
			pi.set_PWM_dutycycle(lb,duty)
			pi.set_PWM_dutycycle(rf,duty)
			pi.set_PWM_dutycycle(lf,0)
			pi.set_PWM_dutycycle(rb,0)
			time.sleep(delay)
			pi.set_PWM_dutycycle(lb,0)
			pi.set_PWM_dutycycle(rf,0)
			

		else:
			if not creep:
				move_forward_by_distance(dist_targ/4)
			else:
				move_forward_by_distance(2)
		

		time.sleep(0.3)
		if pix_y >= 400:
			time.sleep(0.3)
			break

	
	
	move_forward_by_distance(5)
	
	pi.set_PWM_dutycycle(gr,int(round(0.04*255)))
	time.sleep(2)

#-----------------------------------------------------------------------

# # ULTRASONIC SENSOR SETUP-----------------------------------------------
def ping_distance():
	current_duty = pi.get_PWM_dutycycle(gr)
	print("pinging distance")
	maxtime = 0.04
	distances = []
	delay = 0.00001
	while True:
		try:
			while len(distances) < 20: # take 20 readings
				pi.set_PWM_dutycycle(gr,0)
				
				
				pi.write(trig,0)
				time.sleep(0.01)
				
				pi.write(trig,1)
				time.sleep(delay)
				pi.write(trig,0)
				pulse_start = time.time()
				timeout = pulse_start+maxtime # timeout in case sensor fails to hear return echo
			
				while pi.read(echo) == 0 and pulse_start<timeout:
					pulse_start = time.time()
				
				pulse_end = time.time()
				timeout = pulse_end+maxtime
				while pi.read(echo) == 1 and pulse_end<timeout:
					pulse_end = time.time()
				
				pulse_duration = pulse_end - pulse_start
				sonar_distance = pulse_duration*17150
				distances.append(sonar_distance)
				time.sleep(0.01)
			sonar_distance = np.median(distances) # Take median to avoid faulty readings
			break
		except:
			continue
	sonar_distance = round(sonar_distance + 23)
	print("ping distance: " + str(sonar_distance))
	pi.set_PWM_dutycycle(gr,current_duty)
	return sonar_distance

#-----------------------------------------------------------------------

#MOVEMENT ROUTINE FUNCTIONS--------------------------------------------------	

# Position robot set distance from wall for homing
def move_until_distance_from_wall(target_dist):
	current_dist = ping_distance()
	if current_dist > target_dist:
		while current_dist > target_dist:
			travel_dist = max(2,((current_dist-target_dist)/4))
			move_forward_by_distance(travel_dist)
			current_dist = ping_distance()
			time.sleep(0.3)
			
	elif current_dist < target_dist:
		while current_dist < target_dist:
			travel_dist = max(2,((target_dist - current_dist)/4))
			move_backward_by_distance(travel_dist)
			current_dist = ping_distance()
			time.sleep(0.3)

# Home xy position in construction zone
def home_position():
	turn_to_angle(180)
	robot_position[0] = ping_distance()
	turn_to_angle(90)
	robot_position[1] = 305 - ping_distance()

# Travel to middle of construction zone and home position
def go_to_pregoal():
	turn_to_const()
	turn_until_clear_path(None)
	if abs(robot_position[2]-180) > 20:
		distance = (200-robot_position[1])/np.sin(np.deg2rad(robot_position[2]))
	else:
		distance = robot_position[0]-100
	move_forward_by_distance(distance)
	turn_to_angle(180)
	move_until_distance_from_wall(100)
	robot_position[0] = ping_distance()
	turn_to_angle(90)
	move_until_distance_from_wall(100)
	robot_position[1] = 305 - ping_distance()

# Home position on startup
def first_home():
	robot_position[0] = ping_distance()
	turn_to_angle(270)
	robot_position[1] = ping_distance()

#EMAIL ACTIVATION-------------------------------------------------------

# Activate when email recieved 
def checkemail():
	mail = imaplib.IMAP4_SSL('imap.gmail.com')
	mail.login('enpm701zinobile@gmail.com','caafbauozztsduvz')
	mail.list()
	
	
	while True :
		try:
			mail.select("inbox")
			
			result,data = mail.search(None, 'UNSEEN')
			
			ids = data[0]
			id_list = ids.split()
			latest_email_id = id_list[-1]
			result,data = mail.fetch(latest_email_id, "(RFC822)")
			
			if data is None:
				print("waiting...")
			
			if data is not None:
				print("Process initiated")
				break
				
		except IndexError:
			time.sleep(1)
			

	
#-----------------------------------------------------------------------

#MAIN FUNCTION ---------------------------------------------------------
const_home = (100,200)
drop_off_point = (61,244)
checkemail()
blocks_acquired = 0
time.sleep(3)
first_home()
go_to_pregoal()

while blocks_acquired < 9:
	turn_to_angle(45)
	find_path(blocks_acquired)
	time.sleep(1)
	pick_up_block(blocks_acquired)
	sendemail = True
	block_gripped = True
	go_to_pregoal()
	turn_to_angle(150-(5*blocks_acquired))
	move_forward_by_distance(40)
	pi.set_PWM_dutycycle(gr,int(round(0.085*255)))
	time.sleep(2)
	move_backward_by_distance(40)
	home_position()
	blocks_acquired+=1
	block_gripped = False
#-----------------------------------------------------------------------



	

